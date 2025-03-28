class SimpleTracker:
    def camera_control(detection, image_width, image_height):
        center_x = detection.bbox.center.position.x
        center_y = detection.bbox.center.position.y

        rel_x = center_x - (image_width / 2.0)
        rel_y = center_y - (image_height / 2.0)
        
        norm_error_x = rel_x / (image_width / 2.0)
        norm_error_y = rel_y / (image_height / 2.0)
        
        gain_yaw = 0.1
        gain_pitch = 0.1        
        target_yaw = norm_error_x * max_yaw * gain_yaw
        target_pitch = -norm_error_y * max_pitch * gain_pitch

        target_pitch = max(min(target_pitch, 90.0), -90.0)
        target_yaw = max(min(target_yaw, 120.0), -120.0)

        if abs(target_pitch - self.pitch) > 0.1 or abs(target_yaw - self.yaw) > 0.1:
            return
        return target_pitch, target_yaw