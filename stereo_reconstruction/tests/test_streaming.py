"""외부 포트 없이도 ACK 지연·거절·0점 frame·disconnect 후 replay 흐름을 검증."""
import asyncio
import json
from pathlib import Path
import tempfile
import unittest
import hashlib
import numpy as np
from point_stream import RECORD,unpack_packet
try:
    from stream_server import StreamService
    STREAMING=True
except ImportError:
    STREAMING=False


class FakeSource:
    pipeline=None; frames=[0,5]
    def __init__(self,root):
        self.root=Path(root); self.config=dict(chunk_points=2)
        self.records=np.zeros(3,dtype=RECORD); self.records['xyz']=1; self.records['rgba'][:,3]=255
        self.records['point_id']=np.arange(3); self.records['confidence']=.9
    def frame(self,index):
        points=self.records if index==0 else self.records[:0]
        return dict(type='frame_begin',frame_id=self.frames[index],point_count=len(points),total_map_points=3),points
    def finish(self): return dict(map_points=3,binary_sha256=hashlib.sha256(self.records.tobytes()).hexdigest())


class FakeSocket:
    def __init__(self,mode='valid'):
        self.mode=mode; self.queue=asyncio.Queue(); self.binary=[]; self.messages=[]; self.closed=None
    async def send(self,data):
        if isinstance(data,str):
            m=json.loads(data); self.messages.append(m)
            if m['type']=='stream_begin':
                self.stream_id=m['stream_id']; self.queue.put_nowait(json.dumps(dict(type='client_ready',stream_id=self.stream_id)))
        else:
            self.binary.append(data); f,c,_,_=unpack_packet(data)
            if self.mode!='timeout':
                self.queue.put_nowait(json.dumps(dict(type='ack',stream_id=self.stream_id,frame_id=f,chunk_index=c if self.mode=='valid' else c+1)))
    async def recv(self): return await self.queue.get()
    async def close(self,code,reason): self.closed=code


@unittest.skipUnless(STREAMING,'install requirements-streaming.txt for transport tests')
class StreamingTests(unittest.TestCase):
    def test_empty_frame_and_exact_payload(self):
        with tempfile.TemporaryDirectory() as td:
            source=FakeSource(td); service=StreamService(source)
            async def check():
                ws=FakeSocket(); await service.handle(ws)
                self.assertTrue(service.sessions[-1]['complete']); self.assertEqual(len(ws.binary),3)
                self.assertEqual(b''.join(p[32:] for p in ws.binary),source.records.tobytes())
                self.assertEqual(unpack_packet(ws.binary[-1])[:3],(5,0,True))
                self.assertEqual(ws.messages[-1]['type'],'stream_end')
            asyncio.run(check())

    def test_wrong_ack_stops_after_one_chunk_then_reconnect_resets(self):
        with tempfile.TemporaryDirectory() as td:
            service=StreamService(FakeSource(td))
            async def check():
                bad=FakeSocket('wrong'); await service.handle(bad)
                self.assertEqual(len(bad.binary),1); self.assertFalse(service.sessions[-1]['complete'])
                good=FakeSocket(); await service.handle(good)
                self.assertTrue(service.sessions[-1]['complete'])
                self.assertNotEqual(bad.stream_id,good.stream_id); self.assertTrue(good.messages[0]['reset'])
            asyncio.run(check())

    def test_ack_timeout_is_bounded(self):
        with tempfile.TemporaryDirectory() as td:
            service=StreamService(FakeSource(td),ack_timeout=.01)
            async def check():
                ws=FakeSocket('timeout'); await service.handle(ws)
                self.assertEqual(len(ws.binary),1); self.assertEqual(ws.closed,1008)
                self.assertFalse(service.sessions[-1]['complete'])
            asyncio.run(check())

@unittest.skipUnless(STREAMING,'streaming dependency unavailable')
class SurfaceStreamingTests(unittest.TestCase):
    def test_surface_negotiation_empty_and_legacy(self):
        from surface_attributes import SURFACE_RECORD,SURFACE_HEADER
        with tempfile.TemporaryDirectory() as td:
            source=FakeSource(td);source.config['surface_schema']='SRF1'
            for frame,count in [(0,3),(5,0)]:
                folder=Path(td)/f'frames/{frame:010d}';folder.mkdir(parents=True)
                s=np.zeros(count,dtype=SURFACE_RECORD);s['point_id']=np.arange(count)
                (folder/'surface.bin').write_bytes(s.tobytes())
            class SurfaceSocket(FakeSocket):
                async def send(self,data):
                    if isinstance(data,str):
                        m=json.loads(data);self.messages.append(m)
                        if m['type']=='stream_begin':
                            self.stream_id=m['stream_id'];self.queue.put_nowait(json.dumps(dict(type='client_ready',stream_id=self.stream_id,surface_schema='SRF1')))
                    else:
                        self.binary.append(data);h=SURFACE_HEADER.unpack_from(data)
                        self.queue.put_nowait(json.dumps(dict(type='surface_ack' if h[0]==b'SRF1' else 'ack',stream_id=self.stream_id,frame_id=h[3],chunk_index=h[4])))
            async def check():
                service=StreamService(source);ws=SurfaceSocket();await service.handle(ws)
                self.assertTrue(service.sessions[-1]['complete'])
                self.assertEqual([p[:4] for p in ws.binary],[b'STP1',b'SRF1']*3)
                self.assertEqual(len(ws.binary[-1]),32)
                legacy=FakeSocket();await service.handle(legacy)
                self.assertTrue(service.sessions[-1]['complete']);self.assertTrue(all(p[:4]==b'STP1' for p in legacy.binary))
            asyncio.run(check())
