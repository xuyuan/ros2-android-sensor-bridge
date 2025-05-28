class CameraManager {
    constructor() {
        this.cameraStarted = false;
        this.videoTrack = null;
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

            // Create canvas once for reuse
            const canvas = new OffscreenCanvas(640, 480); // Fixed size for performance
            const ctx = canvas.getContext('2d');

            let lastSentTime = 0;
            const desiredFps = 30;
            const frameInterval = 1000 / desiredFps;

            async function processFrame(videoFrame) {
                const bitmap = await createImageBitmap(videoFrame);
                
                // Scale to fit canvas while maintaining aspect ratio
                const scale = Math.min(canvas.width / bitmap.width, canvas.height / bitmap.height);
                const x = (canvas.width - bitmap.width * scale) / 2;
                const y = (canvas.height - bitmap.height * scale) / 2;
                
                // Clear canvas and draw scaled image
                ctx.clearRect(0, 0, canvas.width, canvas.height);
                ctx.drawImage(bitmap, x, y, bitmap.width * scale, bitmap.height * scale);
                
                // Convert to JPEG blob
                const blob = await canvas.convertToBlob({
                    type: 'image/jpeg',
                    quality: 0.7
                });
                
                // Convert blob to base64
                const buffer = await blob.arrayBuffer();
                const base64 = btoa(String.fromCharCode(...new Uint8Array(buffer)));
                return `data:image/jpeg;base64,${base64}`;
            }

            const processFrames = async () => {
                while (true) {
                    const { done, value: videoFrame } = await reader.read();
                    if (done) break;

                    const currentTime = performance.now();
                    if (currentTime - lastSentTime < frameInterval) {
                        videoFrame.close();
                        continue;
                    }

                    if (ws && ws.readyState === WebSocket.OPEN) {
                        try {
                            let lastCameraFrame = await processFrame(videoFrame);
                            ws.send(JSON.stringify({
                                timestamp: Date.now(),
                                camera: lastCameraFrame,
                                width: canvas.width,
                                height: canvas.height
                            }));
                            lastSentTime = currentTime;
                        } catch (err) {
                            console.error('Frame processing error:', err);
                        }
                    }
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
        }
        this.cameraStarted = false;
    }
}

// Export the camera manager
window.CameraManager = CameraManager;
