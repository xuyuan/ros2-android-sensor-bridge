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
  pose: null,
  audio: null
};

// Initialize ROS2 node
async function initRos() {
  await rclnodejs.init();
  const node = rclnodejs.createNode('mobile_sensor_node');
  
  // Create publishers for camera data
  // rosPublishers.compressed = node.createPublisher(
  //   'sensor_msgs/msg/CompressedImage',
  //   'camera/image_raw/compressed',
  //   { qos: { depth: 10 } }
  // );
  rosPublishers.compressed = node.createPublisher(
    'ffmpeg_image_transport_msgs/msg/FFMPEGPacket',
    'camera/image_raw/compressed',
    { qos: { depth: 10 } }
  );

  rosPublishers.cameraInfo = node.createPublisher(
    'sensor_msgs/msg/CameraInfo',
    'camera/camera_info',
    { qos: { depth: 10 } }
  );

  // Add pose publisher
  rosPublishers.pose = node.createPublisher(
    'geometry_msgs/msg/PoseStamped',
    'mobile_sensor/pose',
    { qos: { depth: 10 } }
  );

  // Update audio publisher to use Header and String
  rosPublishers.audio = node.createPublisher(
    'std_msgs/msg/String',
    'mobile_sensor/speech',
    { qos: { depth: 10 } }
  );
  
  // Add GNSS publisher
  rosPublishers.gnss = node.createPublisher(
    'sensor_msgs/msg/NavSatFix',
    'mobile_sensor/gnss',
    { qos: { depth: 10 } }
  );
  
  // Add string subscriber for TTS
  node.createSubscription(
    'std_msgs/msg/String',
    'mobile_sensor/tts',
    (msg) => {
      // Forward the message to all connected TTS clients
      wssTTS.clients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) {
          client.send(msg.data);
        }
      });
    },
    { qos: { depth: 10 } }
  );
  
  console.log('ROS2 node initialized with camera, pose publishers and TTS subscriber');
  return node;
}

// Serve static files from the "public" folder
app.use(express.static('public'));
app.use(express.json());

// Serve index.html on GET /
app.get('/', (req, res) => {
  res.sendFile(path.join(__dirname, 'public', 'index.html'));
});

const options = {
    key: fs.readFileSync(path.join(__dirname, 'key.pem')),
    cert: fs.readFileSync(path.join(__dirname, 'cert.pem')),
};


// Create HTTPS server
const server = https.createServer(options, app);

// Create WebSocket servers for different data types
const wssPose = new WebSocket.Server({ noServer: true });
const wssCamera = new WebSocket.Server({ noServer: true });
const wssTTS = new WebSocket.Server({ noServer: true });
const wssAudio = new WebSocket.Server({ noServer: true });
const wssGNSS = new WebSocket.Server({ noServer: true });

// Handle upgrade requests to separate WebSocket connections
server.on('upgrade', (request, socket, head) => {
  const pathname = new URL(request.url, `http://${request.headers.host}`).pathname;

  switch (pathname) {
    case '/tts':
      wssTTS.handleUpgrade(request, socket, head, (ws) => {
        wssTTS.emit('connection', ws, request);
      });
      break;
    case '/pose':
      wssPose.handleUpgrade(request, socket, head, (ws) => {
        wssPose.emit('connection', ws, request);
      });
      break;
    case '/camera':
      wssCamera.handleUpgrade(request, socket, head, (ws) => {
        wssCamera.emit('connection', ws, request);
      });
      break;
    case '/audio':
      wssAudio.handleUpgrade(request, socket, head, (ws) => {
        wssAudio.emit('connection', ws, request);
      });
      break;
    case '/gnss':
      wssGNSS.handleUpgrade(request, socket, head, (ws) => {
        wssGNSS.emit('connection', ws, request);
      });
      break;
    default:
      socket.destroy();
  }
});

// Handle pose data WebSocket connections
wssPose.on('connection', (ws) => {
  console.log('New pose data WebSocket client connected');
  ws.on('message', async (message) => {
    try {
      const data = JSON.parse(message);
      if (data.pose) {
        // Generate current timestamp
        const now = Date.now();
        const stamp = {
          sec: Math.floor(now / 1000),
          nanosec: (now % 1000) * 1000000
        };
        
        // Create and publish ROS2 pose message
        const poseMsg = {
          header: {
            stamp: stamp,
            frame_id: 'odom'
          },
          pose: data.pose
        };
        rosPublishers.pose.publish(poseMsg);
      }
    } catch (err) {
      console.error('Error processing pose message:', err);
    }
  });
});

// Handle GNSS data WebSocket connections
wssGNSS.on('connection', (ws) => {
  console.log('New GNSS data WebSocket client connected');
  ws.on('message', async (message) => {
    try {
      const data = JSON.parse(message);
      if (data.position) {
        const stamp = {
          sec: Math.floor(data.timestamp / 1000),
          nanosec: (data.timestamp % 1000) * 1000000
        };
        
        const accuracy2 = data.position.coords.accuracy *  data.position.coords.accuracy;
        
        let altitude = 0;
        let status = 0; // STATUS_FIX
        if (data.position.coords.altitude) {
          altitude = data.position.coords.altitude;
          status = 1; // STATUS_SBAS_FIX
        }
        
        // Create and publish ROS2 pose message
        const msg = {
          header: {
            stamp: stamp,
            frame_id: 'gnss'
          },
          latitude: data.position.coords.latitude,
          longitude: data.position.coords.longitude,
          altitude: altitude,
          status: {status: status},
          position_covariance_type: 1,
          position_covariance: [accuracy2, 0, 0, 0, accuracy2, 0, 0, 0, accuracy2]
        };
        rosPublishers.gnss.publish(msg);
      }
    } catch (err) {
      console.error('Error processing GNSS message:', err);
    }
  });
});

// Handle camera data WebSocket connections
wssCamera.on('connection', (ws) => {
  console.log('New camera data WebSocket client connected');
  ws.on('message', async (message) => {
    try {
      const data = JSON.parse(message);
      if (data.camera) {
        // Process camera frame
        // console.log('Received camera frame:', 
        //   `${data.camera.substring(0, 50)}... [${data.camera.length} chars]`);
        
        try {
          // Convert base64 string to binary data
          // const base64Data = data.camera.split(',')[1]; // Remove data URL prefix if present
          // const imageBuffer = Buffer.from(base64Data, 'base64');
          
          // Extract image dimensions if available, or use defaults
          const width = data.width || 640;
          const height = data.height || 480;
          
          // Generate current timestamp
          const now = Date.now();
          const stamp = {
            sec: Math.floor(now / 1000),
            nanosec: (now % 1000) * 1000000
          };
          
          // Create standard header
          const header = {
            stamp: stamp,
            frame_id: 'camera_frame'
          };
          
          // 1. Publish CompressedImage message (most efficient for JPEG data)
          // const compressedMsg = {
          //   header: header,
          //   format: 'jpeg',
          //   data: Array.from(imageBuffer)
          // };
          const compressedMsg = {
            header: header,
            width: width,
            height: height,
            encoding: 'h264',
            // pts: BigInt(0),
            // flags: 0,
            is_bigendian: false,
            data: Array.from(data.camera)
          };
          rosPublishers.compressed.publish(compressedMsg);
          console.log('ROS2 CompressedImage message published');
          
          // 2. Publish CameraInfo message
          const cameraInfoMsg = {
            header: header,
            height: height,
            width: width,
            distortion_model: 'plumb_bob',
            d: [0.0, 0.0, 0.0, 0.0, 0.0],  // Default distortion coefficients
            k: [  // Default intrinsic camera matrix
              width, 0.0, width/2,
              0.0, height, height/2,
              0.0, 0.0, 1.0
            ],
            r: [  // Default rectification matrix (identity)
              1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0
            ],
            p: [  // Default projection matrix
              width, 0.0, width/2, 0.0,
              0.0, height, height/2, 0.0,
              0.0, 0.0, 1.0, 0.0
            ],
            binning_x: 0,
            binning_y: 0,
            roi: {
              x_offset: 0,
              y_offset: 0,
              height: height,
              width: width,
              do_rectify: false
            }
          };
          rosPublishers.cameraInfo.publish(cameraInfoMsg);
          
          // console.log('Published camera data to ROS2 topics');
        } catch (error) {
          console.error('Error publishing to ROS2:', error);
        }
      }
    } catch (err) {
      console.error('Error processing camera message:', err);
    }
  });
});

// Handle TTS WebSocket connections
wssTTS.on('connection', (ws) => {
  console.log('New TTS WebSocket client connected');
  ttsClients.add(ws);
  
  ws.on('close', () => {
    ttsClients.delete(ws);
  });
});

// Update audio WebSocket handler
wssAudio.on('connection', (ws) => {
  console.log('New audio WebSocket client connected');
  ws.on('message', async (message) => {
    try {
      const data = JSON.parse(message);
      if (data.transcription) {
        console.log(`Transcription received: "${data.transcription}"`);
        
        // Create timestamped message with header
        const now = Date.now();
        const msg = {
          header: {
            stamp: {
              sec: Math.floor(now / 1000),
              nanosec: (now % 1000) * 1000000
            },
            frame_id: 'audio_frame'
          },
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
  
  wssPose.clients.forEach(client => client.close());
  wssCamera.clients.forEach(client => client.close());
  wssTTS.clients.forEach(client => client.close());
  wssAudio.clients.forEach(client => client.close());
  wssGNSS.clients.forEach(client => client.close());

  if (rclnodejs.isShutdown() === false) {
    rclnodejs.shutdown();
    console.log('ROS2 node shut down');
  }
  
  server.close(() => {
    console.log('HTTPS server closed');
    process.exit(0);
  });
}

// Listen for termination signals
process.on('SIGINT', shutdown);
process.on('SIGTERM', shutdown);
process.on('uncaughtException', (err) => {
  console.error('Uncaught exception:', err);
  shutdown();
});

// Initialize ROS2 and start server
initRos().then((node) => {
  const port = process.env.PORT || 4000;
  server.listen(port, () => {
    console.log(`HTTPS server running at https://localhost:${port}`);
    // Start ROS2 spinning
    rclnodejs.spin(node);
  });
}).catch((err) => {
  console.error('Failed to initialize ROS2:', err);
  process.exit(1);
});
