"""직접 SGM pipeline의 신규 voxel점만 전송하는 실제 WebSocket 서버.

한 chunk ACK 후 다음 chunk. live는 frame 계산 직후 전송; replay는 같은 저장 bytes.
깊이/포즈/voxel은 자체 구현, RFC6455 transport만 websockets13.1을 사용한다.
"""
import argparse
import asyncio
import json
from pathlib import Path
import uuid
import numpy as np
from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed
from point_stream import RECORD,packetize,unpack_packet
from surface_attributes import SURFACE_RECORD,packetize_surface
from kitti_pipeline import add_arguments,from_arguments,write_json


class RunSource:
    def __init__(self,pipeline=None,replay=None):
        self.pipeline=pipeline; self.root=Path(replay) if replay else pipeline.output
        self.config=json.loads((self.root/'config.json').read_text())
        self.frames=self.config['frames']; self.summary=None
        if replay:
            self.summary=json.loads((self.root/'summary.json').read_text())
            if len(self.summary['frames'])!=len(self.frames): raise ValueError('불완전 run은 replay할 수 없습니다.')
    def frame(self,index):
        if self.pipeline and self.pipeline.next_index<=index: return self.pipeline.step()
        folder=self.root/f'frames/{self.frames[index]:010d}'
        return json.loads((folder/'frame.json').read_text()),np.frombuffer((folder/'new_points.bin').read_bytes(),dtype=RECORD)
    def finish(self):
        if self.summary is None: self.summary=self.pipeline.finish()
        return self.summary


class StreamService:
    def __init__(self,source,ack_timeout=30.):
        if ack_timeout<=0: raise ValueError('ack timeout은 양수여야 합니다.')
        self.source=source; self.ack_timeout=ack_timeout; self.busy=False
        history=self.source.root/'server_sessions.json'
        self.sessions=json.loads(history.read_text()) if history.exists() else []
    async def expected(self,ws,message_type,stream_id,**fields):
        raw=await asyncio.wait_for(ws.recv(),timeout=self.ack_timeout)
        if not isinstance(raw,str): raise ValueError('client 제어 메시지는 JSON text여야 합니다.')
        message=json.loads(raw)
        if not isinstance(message,dict) or message.get('type')!=message_type or message.get('stream_id')!=stream_id:
            raise ValueError('잘못된 제어 메시지 또는 stream_id')
        if any(message.get(k)!=v for k,v in fields.items()): raise ValueError('잘못된 ACK frame/chunk')
        return message
    async def handle(self,ws):
        if self.busy:
            await ws.close(1013,'One active client supported; retry after current stream.'); return
        self.busy=True; stream_id=str(uuid.uuid4())
        log=dict(stream_id=stream_id,source_mode='live' if self.source.pipeline else 'replay',complete=False,chunks=0,points=0,binary_bytes=0)
        async def send(value): await ws.send(json.dumps(value,ensure_ascii=False,allow_nan=False))
        try:
            await send(dict(type='stream_begin',schema='stereo-map-stream/1',packet_schema='STP1',
                stream_id=stream_id,reset=True,coordinate_frame='world_first_left_camera',position_unit='metre',
                pose_applied=True,point_id_scope='stream_global',point_stride=32,byte_order='little-endian',
                world_reference_frame=0,world_axes='+X right,+Y down,+Z forward at KITTI frame0 left camera',
                suggested_viewer_from_world_column_major=[1,0,0,0,0,-1,0,0,0,0,-1,0,0,0,0,1],
                frame_ids=self.source.frames,ack_required=True,ack_timeout_seconds=self.ack_timeout,
                surface_schema=self.source.config.get('surface_schema'),
                source_mode='live' if self.source.pipeline else 'replay',config=self.source.config))
            ready=await self.expected(ws,'client_ready',stream_id)
            use_surface=ready.get('surface_schema')=='SRF1'
            if use_surface and self.source.config.get('surface_schema')!='SRF1': raise ValueError('Surface unavailable in this run')
            for index in range(len(self.source.frames)):
                # Python3.8 호환, CPU 작업은 executor에서 수행해 ping/event loop 유지.
                meta,records=await asyncio.get_running_loop().run_in_executor(None,self.source.frame,index)
                await send(dict(meta,stream_id=stream_id))
                surface_packets=None
                if use_surface:
                    raw=(self.source.root/f'frames/{meta["frame_id"]:010d}/surface.bin').read_bytes()
                    surf=np.frombuffer(raw,dtype=SURFACE_RECORD)
                    if len(surf)!=len(records) or not np.array_equal(surf['point_id'],records['point_id']): raise ValueError('Surface/point ID mismatch')
                    surface_packets=iter(packetize_surface(surf,meta['frame_id'],self.source.config['chunk_points']))
                for packet in packetize(records,meta['frame_id'],self.source.config['chunk_points']):
                    frame,chunk,last,points=unpack_packet(packet)
                    await ws.send(packet)
                    await self.expected(ws,'ack',stream_id,frame_id=frame,chunk_index=chunk)
                    if surface_packets is not None:
                        await ws.send(next(surface_packets))
                        await self.expected(ws,'surface_ack',stream_id,frame_id=frame,chunk_index=chunk)
                    log['chunks']+=1; log['points']+=len(points); log['binary_bytes']+=len(packet)
                await send(dict(type='frame_end',stream_id=stream_id,frame_id=meta['frame_id'],
                    added_points=len(records),total_map_points=meta['total_map_points']))
            summary=await asyncio.get_running_loop().run_in_executor(None,self.source.finish)
            await send(dict(type='stream_end',stream_id=stream_id,complete=True,map_points=summary['map_points'],
                sha256=summary['binary_sha256'],surface_sha256=summary.get('surface_sha256') if use_surface else None,frame_count=len(self.source.frames)))
            log['complete']=True
        except (ValueError,asyncio.TimeoutError,ConnectionClosed) as error:
            log['error']=type(error).__name__+': '+str(error)
            try:
                await send(dict(type='error',stream_id=stream_id,message=log['error']))
                await ws.close(1008,'Invalid control message, disconnect, or ACK timeout')
            except ConnectionClosed: pass
        except Exception as error:
            log['error']=type(error).__name__+': '+str(error)
            try:
                await send(dict(type='error',stream_id=stream_id,message=log['error']))
                await ws.close(1011,'Pipeline failed')
            except ConnectionClosed: pass
        finally:
            self.sessions.append(log); self.busy=False
            write_json(self.source.root/'server_sessions.json',self.sessions)


async def run_server(source,host='127.0.0.1',port=8765,once=False,ack_timeout=30.):
    service=StreamService(source,ack_timeout); finished=asyncio.Event()
    async def handler(ws):
        rejected_busy=service.busy
        try: await service.handle(ws)
        finally:
            if once and not rejected_busy: finished.set()
    async with serve(handler,host,port,max_size=4096,max_queue=1,write_limit=65536,compression=None) as server:
        actual=server.sockets[0].getsockname()[1]
        print(f'WebSocket ready: ws://{host}:{actual}',flush=True)
        if once: await finished.wait()
        else: await asyncio.Future()
    return service.sessions


def main():
    p=argparse.ArgumentParser(description=__doc__); modes=p.add_subparsers(dest='mode',required=True)
    live=modes.add_parser('live'); add_arguments(live)
    replay=modes.add_parser('replay'); replay.add_argument('--run',required=True)
    for sub in (live,replay):
        sub.add_argument('--host',default='127.0.0.1'); sub.add_argument('--port',type=int,default=8765)
        sub.add_argument('--once',action='store_true'); sub.add_argument('--ack-timeout',type=float,default=30.)
    a=p.parse_args(); source=RunSource(pipeline=from_arguments(a)) if a.mode=='live' else RunSource(replay=a.run)
    asyncio.run(run_server(source,a.host,a.port,a.once,a.ack_timeout))


if __name__=='__main__': main()
