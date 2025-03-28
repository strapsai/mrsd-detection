class GPSEstimator:
    """
    Unified GPS estimator that converts camera detections to real-world coordinates
    by considering camera parameters (camera type:intrinsic matrix, zoom), drone state(GPS, altitude, 3D orientation), and detection information(bounding box).
    """
    def __init__(self, config: Dict):