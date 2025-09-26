// node.js (변경된 부분 위주 전체)
const https = require('https');
const fs = require('fs');
const express = require('express');
const path = require('path');
const WebSocket = require('ws');
const rclnodejs = require('rclnodejs');

const app = express();
let ttsClients = new Set();
let rosPublishers = {
  compressed: null,
  cameraInfo: null,
  poseStamped: null,
  audio: null
};

async function initRos() {
  await rclnodejs.init();
  const node = rclnodejs.createNode('mobile_sensor_node');

  // Camera publishers (절대 토픽)
  rosPublishers.compressed = node.createPublisher(
    'sensor_msgs/msg/CompressedImage',
    '/camera/image_raw/compressed',
    { qos: { depth: 10 } }
  );

  rosPublishers.cameraInfo = node.createPublisher(
    'sensor_msgs/msg/CameraInfo',
    '/camera/camera_info',
    { qos: { depth: 10 } }
  );

  // PoseStamped publisher (절대 토픽)
  rosPublishers.poseStamped = node.createPublisher(
    'geometry_msgs/msg/PoseStamped',
    '/mobile_sensor/pose_stamped',
    { qos: { depth: 10 } }
  );

  // Audio publisher (절대 토픽)
  rosPublishers.audio = node.createPublisher(
    'std_msgs/msg/String',
    '/mobile_sensor/speech',
    { qos: { depth: 10 } }
  );

  // TTS subscriber (절대 토픽)
  node.createSubscription(
    'std_msgs/msg/String',
    '/mobile_sensor/tts',
    (msg) => {
      wssTTS.clients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) client.send(msg.data);
      });
    },
    { qos: { depth: 10 } }
  );

  console.log('ROS2 node initialized with camera, pose_stamped publishers and TTS subscriber');
  return node;
}

app.use(express.static('public'));
app.use(express.json());

app.get('/', (req, res) => {
  res.sendFile(path.join(__dirname, 'public', 'index.html'));
});

const options = {
  key: fs.readFileSync(path.join(__dirname, 'key.pem')),
  cert: fs.readFileSync(path.join(__dirname, 'cert.pem')),
};

const server = https.createServer(options, app);

const wssPose = new WebSocket.Server({ noServer: true });
const wssCamera = new WebSocket.Server({ noServer: true });
const wssTTS = new WebSocket.Server({ noServer: true });
const wssAudio = new WebSocket.Server({ noServer: true });

server.on('upgrade', (request, socket, head) => {
  const pathname = new URL(request.url, `http://${request.headers.host}`).pathname;
  switch (pathname) {
    case '/tts':    wssTTS.handleUpgrade(request, socket, head, ws => wssTTS.emit('connection', ws, request)); break;
    case '/pose':   wssPose.handleUpgrade(request, socket, head, ws => wssPose.emit('connection', ws, request)); break;
    case '/camera': wssCamera.handleUpgrade(request, socket, head, ws => wssCamera.emit('connection', ws, request)); break;
    case '/audio':  wssAudio.handleUpgrade(request, socket, head, ws => wssAudio.emit('connection', ws, request)); break;
    default: socket.destroy();
  }
});

// PoseStamped over WS → ROS
wssPose.on('connection', (ws) => {
  console.log('New pose data WebSocket client connected');
  ws.on('message', async (message) => {
    try {
      const data = JSON.parse(message);
      if (data.pose) {
        const now = Date.now();
        const stamp = data.stamp
          ? data.stamp
          : { sec: Math.floor(now / 1000), nanosec: (now % 1000) * 1e6 };

        const frame_id = data.frame_id || 'phone_frame';

        const poseStampedMsg = {
          header: { stamp, frame_id },
          pose: {
            position: {
              x: data.pose.position.x,
              y: data.pose.position.y,
              z: data.pose.position.z
            },
            orientation: {
              x: data.pose.orientation.x,
              y: data.pose.orientation.y,
              z: data.pose.orientation.z,
              w: data.pose.orientation.w
            }
          }
        };
        rosPublishers.poseStamped.publish(poseStampedMsg);
      }
    } catch (err) {
      console.error('Error processing pose message:', err);
    }
  });
});

// Camera over WS → ROS (CompressedImage + CameraInfo; 같은 stamp 사용)
wssCamera.on('connection', (ws) => {
  console.log('New camera data WebSocket client connected');
  ws.on('message', async (message) => {
    try {
      const data = JSON.parse(message);
      if (data.camera) {
        try {
          const base64Data = data.camera.split(',')[1];
          const imageBuffer = Buffer.from(base64Data, 'base64');

          const width = data.width || 640;
          const height = data.height || 480;

          const now = Date.now();
          const stamp = {
            sec: Math.floor(now / 1000),
            nanosec: (now % 1000) * 1e6
          };
          const header = { stamp, frame_id: data.frame_id || 'camera_frame' };

          const compressedMsg = { header, format: 'jpeg', data: Array.from(imageBuffer) };
          rosPublishers.compressed.publish(compressedMsg);

          const cameraInfoMsg = {
            header,
            height, width,
            distortion_model: 'plumb_bob',
            d: [0,0,0,0,0],
            k: [ width,0, width/2, 0, height, height/2, 0,0,1 ],
            r: [ 1,0,0, 0,1,0, 0,0,1 ],
            p: [ width,0, width/2,0, 0,height,height/2,0, 0,0,1,0 ],
            binning_x: 0, binning_y: 0,
            roi: { x_offset:0, y_offset:0, height, width, do_rectify:false }
          };
          rosPublishers.cameraInfo.publish(cameraInfoMsg);
        } catch (error) {
          console.error('Error publishing camera to ROS2:', error);
        }
      }
    } catch (err) {
      console.error('Error processing camera message:', err);
    }
  });
});

// TTS
wssTTS.on('connection', (ws) => {
  console.log('New TTS WebSocket client connected');
  ttsClients.add(ws);
  ws.on('close', () => ttsClients.delete(ws));
});

// ASR transcription
wssAudio.on('connection', (ws) => {
  console.log('New audio WebSocket client connected');
  ws.on('message', async (message) => {
    try {
      const data = JSON.parse(message);
      if (data.transcription) {
        const now = Date.now();
        const msg = {
          header: { stamp: { sec: Math.floor(now/1000), nanosec: (now%1000)*1e6 }, frame_id: 'audio_frame' },
          data: data.transcription
        };
        rosPublishers.audio.publish(msg);
      }
    } catch (err) {
      console.error('Error processing audio message:', err);
    }
  });
});

function shutdown() {
  console.log('Shutting down server...');
  [wssPose, wssCamera, wssTTS, wssAudio].forEach(s => s.clients.forEach(c => c.close()));
  if (!rclnodejs.isShutdown()) rclnodejs.shutdown();
  server.close(() => process.exit(0));
}

process.on('SIGINT', shutdown);
process.on('SIGTERM', shutdown);
process.on('uncaughtException', (err) => { console.error('Uncaught exception:', err); shutdown(); });

initRos().then((node) => {
  const port = process.env.PORT || 4000;
  server.listen(port, () => {
    console.log(`HTTPS server running at https://localhost:${port}`);
    rclnodejs.spin(node);
  });
}).catch((err) => {
  console.error('Failed to initialize ROS2:', err);
  process.exit(1);
});
