class CameraManager {
    constructor() {
        this.cameraStarted = false;
        this.videoTrack = null;
        this.encoder = null;
    }

    async startCamera(ws, isSessionActive) {
        if (this.cameraStarted || !isSessionActive) return;
        try {
            if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
                throw new Error('getUserMedia API not supported');
            }

            const stream = await navigator.mediaDevices.getUserMedia({
                video: { facingMode: "user" },
            });
            this.videoTrack = stream.getVideoTracks()[0];

            const trackProcessor = new MediaStreamTrackProcessor({ track: this.videoTrack });
            const reader = trackProcessor.readable.getReader();
            let decoderConfig = null;

            function handleChunk(chunk, metadata) {
                if (metadata.decoderConfig) {
                    // Decoder needs to be configured (or reconfigured) with new parameters
                    // when metadata has a new decoderConfig.
                    // Usually it happens in the beginning or when the encoder has a new
                    // codec specific binary configuration. (VideoDecoderConfig.description).
                    decoderConfig = metadata.decoderConfig;
                }

                // actual bytes of encoded data
                const chunkData = new Uint8Array(chunk.byteLength);
                chunk.copyTo(chunkData);

                if (ws && ws.readyState === WebSocket.OPEN) {
                        try {
                            ws.send(JSON.stringify({
                            timestamp: Date.now(),
                            camera: chunkData,
                            width: 640,
                            height: 480,
                            decoderConfig: decoderConfig.description,
                        }));
                            
                        } catch (err) {
                            console.error('websocket error:', err);
                        }
                    }
            }

            const init = {
                output: handleChunk,
                error: (e) => {
                    console.log(e.message);
                },
            };

            const config = {
                codec: "vp8",
                width: 640,
                height: 480,
                bitrate: 2_000_000, // 2 Mbps
                framerate: 30,
            };

            const { supported } = await VideoEncoder.isConfigSupported(config);
            if (supported) {
                this.encoder = new VideoEncoder(init);
                this.encoder.configure(config);
            } else {
                // Try another config.
                alert('VideoEncoder config not supported!');
                return;
            }

            let frameCounter = 0;
            let lastSentTime = 0;
            const desiredFps = 30;
            const frameInterval = 1000 / desiredFps;

            const processFrames = async () => {
                while (true) {
                    const { done, value: videoFrame } = await reader.read();
                    if (done) break;

                    const currentTime = performance.now();
                    if (currentTime - lastSentTime < frameInterval) {
                        videoFrame.close();
                        continue;
                    }

                    if (this.encoder.encodeQueueSize < 2) {
                        // Too many frames in flight, encoder is overwhelmed
                        // let's drop this frame.
                        frameCounter++;
                        const keyFrame = frameCounter % 150 == 0;
                        this.encoder.encode(videoFrame, { keyFrame });
                    }
                    lastSentTime = currentTime;
                    videoFrame.close();
                }
            };

            processFrames().catch(console.error);
            this.cameraStarted = true;
        } catch (err) {
            console.error('Camera error:', err);
            alert('Camera access failed: ' + err.message);
        }
    }

    stopCamera() {
        if (this.videoTrack) {
            this.videoTrack.stop();
            this.videoTrack = null;
            this.encoder = null;
        }
        this.cameraStarted = false;
    }
}

// Export the camera manager
window.CameraManager = CameraManager;
