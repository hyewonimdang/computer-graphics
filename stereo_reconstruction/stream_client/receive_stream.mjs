/** 실제 WebSocket + 기존 STP1 decoder 검증용 Node22+ client. HTML/렌더러 아님. */
import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import assert from 'node:assert/strict';
import { readPointPacket } from './read_points.mjs';
import { readSurfacePacket } from './read_surface.mjs';

const url = process.argv[2] || 'ws://127.0.0.1:8765';
const output = process.argv[3] || 'received_stream';
fs.mkdirSync(output, { recursive: true });
const report = await new Promise((resolve, reject) => {
  const ws = new WebSocket(url);
  ws.binaryType = 'arraybuffer';
  const chunks = [], frames = [], seen = new Set(), surfaces = [];
  let pendingSurface=null, surfaceCount=0;
  let stream = null, current = null, count = 0, binaryBytes = 0, finished = false;
  const timeout = setTimeout(() => { ws.close(); reject(new Error('Whole stream timeout')); }, 300000);
  const send = value => ws.send(JSON.stringify(value));
  ws.onmessage = event => {
    try {
      if (typeof event.data === 'string') {
        const m = JSON.parse(event.data);
        if (m.type === 'stream_begin') {
          assert.equal(stream, null); assert.equal(m.reset, true);
          assert.equal(m.coordinate_frame, 'world_first_left_camera'); assert.equal(m.position_unit, 'metre');
          assert.equal(m.pose_applied, true); assert.equal(m.point_stride, 32);
          stream = m;
          send({ type: 'client_ready', stream_id: stream.stream_id, surface_schema: m.surface_schema==='SRF1' ? 'SRF1' : null });
        } else if (m.type === 'frame_begin') {
          assert.equal(m.stream_id, stream.stream_id); assert.equal(current, null);
          assert.equal(m.frame_id, stream.frame_ids[frames.length]);
          current = { metadata: m, received: 0, chunks: 0, last: false };
        } else if (m.type === 'frame_end') {
          assert.equal(m.stream_id, stream.stream_id); assert.ok(current && current.last); assert.equal(pendingSurface,null);
          assert.equal(m.frame_id, current.metadata.frame_id);
          assert.equal(current.received, current.metadata.point_count); assert.equal(current.received, m.added_points);
          assert.equal(current.chunks, current.metadata.chunk_count); assert.equal(count, m.total_map_points);
          frames.push({ ...current.metadata, received_points: current.received, received_chunks: current.chunks });
          console.log(`frame ${m.frame_id}: received ${current.received} new points, total ${count}`);
          current = null;
        } else if (m.type === 'stream_end') {
          assert.equal(m.stream_id, stream.stream_id); assert.equal(current, null); assert.equal(m.complete, true);
          assert.equal(m.map_points, count); assert.equal(m.frame_count, frames.length);
          const data = Buffer.concat(chunks);
          const hash = crypto.createHash('sha256').update(data).digest('hex');
          assert.equal(hash, m.sha256); assert.equal(data.length, count * 32);
          fs.writeFileSync(path.join(output, 'received_points.bin'), data);
          if(stream.surface_schema==='SRF1') {
            assert.equal(surfaceCount,count); const surfaceData=Buffer.concat(surfaces);
            assert.equal(crypto.createHash('sha256').update(surfaceData).digest('hex'),m.surface_sha256);
            fs.writeFileSync(path.join(output,'received_surface.bin'),surfaceData);
          }
          finished = true; clearTimeout(timeout); ws.close();
          resolve({ status: 'PASS', url, node: process.version, stream, frames, map_points: count,
            surface_points:surfaceCount, surface_sha256:m.surface_sha256, chunks: seen.size, binary_bytes_including_headers: binaryBytes, payload_sha256: hash,
            validation: 'Existing JS decoder; global IDs; no repeated chunks; counts; server byte-for-byte SHA256' });
        } else if (m.type === 'error') throw new Error(m.message);
        else throw new Error(`Unknown message ${m.type}`);
      } else {
        if(pendingSurface) {
          const surface=readSurfacePacket(event.data);
          assert.equal(surface.frameId,pendingSurface.frameId); assert.equal(surface.chunkIndex,pendingSurface.chunkIndex);
          assert.equal(surface.isLast,pendingSurface.isLast); assert.deepEqual(surface.pointIds,pendingSurface.pointIds);
          surfaces.push(Buffer.from(event.data,32));surfaceCount+=surface.count;
          send({type:'surface_ack',stream_id:stream.stream_id,frame_id:surface.frameId,chunk_index:surface.chunkIndex});
          pendingSurface=null;return;
        }
        assert.ok(stream && current && !current.last);
        const points = readPointPacket(event.data);
        assert.equal(points.frameId, current.metadata.frame_id); assert.equal(points.chunkIndex, current.chunks);
        if(stream.surface_schema==='SRF1') pendingSurface=points;
        const key = `${points.frameId}:${points.chunkIndex}`;
        assert.equal(seen.has(key), false); seen.add(key);
        for (let i = 0; i < points.count; i++) assert.equal(points.pointIds[i], count + i);
        // B는 여기서 GPU에 points.positions/rgb/confidence를 추가한 다음 ACK한다.
        // positions는 이미 world/metre. T_world_from_observation_camera를 다시 적용하면 안 됨.
        chunks.push(Buffer.from(event.data, 32));
        binaryBytes += event.data.byteLength; count += points.count;
        current.received += points.count; current.chunks++; current.last = points.isLast;
        send({ type: 'ack', stream_id: stream.stream_id, frame_id: points.frameId, chunk_index: points.chunkIndex });
      }
    } catch (error) { clearTimeout(timeout); ws.close(); reject(error); }
  };
  ws.onerror = () => { clearTimeout(timeout); reject(new Error('WebSocket connection error')); };
  ws.onclose = () => { clearTimeout(timeout); if (!finished) reject(new Error('Stream closed before stream_end')); };
});
fs.writeFileSync(path.join(output, 'validation.json'), JSON.stringify(report, null, 2) + '\n');
console.log(`PASS: ${report.map_points} points, ${report.chunks} chunks, SHA256 ${report.payload_sha256}`);
