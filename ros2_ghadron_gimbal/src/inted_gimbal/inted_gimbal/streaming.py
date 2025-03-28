from cv_bridge import CvBridge
import cv2
import subprocess
import numpy as np
import time
import threading

#class to handle all the rtsp retrieval functions and stuff
class Streaming:
    rtsp_url = 'rtsp://10.3.1.124:8554/ghadron'
    WIDTH = 1280
    HEIGHT = 720
    retry_max = 10
    retry_delay = 2.0
    fps = 10
    is_running = True
    retry_count = 0
    process = None
    frame_count = 0
    last_log_time = time.time()
    last_frame_time = 0
    bridge = CvBridge()
    
    def get_ffmpeg_cmd(self):
        return [
            "ffmpeg",
            "-fflags", "nobuffer",
            "-flags", "low_delay",
            "-rtsp_transport", "tcp",
            "-stimeout", "5000000",    
            "-use_wallclock_as_timestamps", "1",
            "-avioflags", "direct",
            "-flush_packets", "1",
            "-probesize", "32",        # Reduce initial buffering
            "-analyzeduration", "0",   # Skip detailed stream analysis
            "-thread_queue_size", "512", # Increase thread queue size
            "-hwaccel", "auto",        # Use hardware acceleration if available
            "-i", self.rtsp_url,
            "-vsync", "0",
            "-copyts",
            "-vf", f"fps={self.fps},scale={self.WIDTH}:{self.HEIGHT}",
            "-pix_fmt", "bgr24",
            "-f", "rawvideo",
            "-"
        ]

    def start_ffmpeg(self):
        try:
            cmd = self.get_ffmpeg_cmd()
            
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=10**8
            )
            
            if self.process.poll() is None:
                return True
            else:
                return False
                
        except Exception as e:
            return False

    def retrieve_image(self):
        frame_size = self.WIDTH * self.HEIGHT * 3
        
        # Pre-allocate the buffer for better performance
        frame_buffer = bytearray(frame_size)
        view = memoryview(frame_buffer)
        
        #checking if the ffmpeg is running
        if not self.process or self.process.poll() is not None:
            retry_wait = self.retry_delay * (1.5 ** min(self.retry_count, 10))
            self.retry_count += 1
            
            if self.retry_count > 1:
                time.sleep(retry_wait)
            
            if not self.start_ffmpeg():
                return
            else:
                self.retry_count = 0

        
        try:
            # More efficient reading using pre-allocated buffer
            bytes_read = 0
            while bytes_read < frame_size and self.is_running:
                chunk = self.process.stdout.read(frame_size - bytes_read)
                if not chunk:
                    break
                view[bytes_read:bytes_read+len(chunk)] = chunk
                bytes_read += len(chunk)
            
            if bytes_read != frame_size:
                print("Misalignment in bytes!")
                return

            #return the image frame
            frame = np.frombuffer(frame_buffer, np.uint8).reshape((self.HEIGHT, self.WIDTH, 3))
            return frame
            
        except Exception as e:
            pass

    def shutdown(self):
        self.is_running = False
        
        if self.process and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()

        
        