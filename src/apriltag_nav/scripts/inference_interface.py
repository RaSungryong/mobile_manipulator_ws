#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import numpy as np
import cv2
from typing import Optional

import rospy
import torchvision.transforms as transforms
from PIL import Image as PILImage
import onnxruntime as ort


class InferenceInterface:
    """ONNX Runtime (CPU) inference interface for Ra surface roughness prediction."""

    def __init__(self):
        self.session = None
        self.input_name = None

        self.transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Resize((900, 900)),
            transforms.Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])
        ])

    def load_model(self, model_path: str) -> bool:
        try:
            rospy.loginfo("[Inference] Loading ONNX model...")
            sess_options = ort.SessionOptions()
            sess_options.intra_op_num_threads = max(1, os.cpu_count() // 2)
            sess_options.graph_optimization_level = (
                ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            )
            self.session = ort.InferenceSession(
                model_path,
                sess_options=sess_options,
                providers=["CPUExecutionProvider"],
            )
            self.input_name = self.session.get_inputs()[0].name

            # Warm up the model with dummy input
            dummy = np.random.randn(1, 3, 900, 900).astype(np.float32)
            for _ in range(3):
                self.session.run(None, {self.input_name: dummy})

            rospy.loginfo("[Inference] ONNX model loaded successfully")
            return True
        except Exception as e:
            rospy.logerr(f"[Inference] Failed to load: {e}")
            return False

    def infer(self, image) -> Optional[float]:
        if self.session is None:
            return None
        try:
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(image_rgb)
            input_np = (
                self.transform(pil_image)
                .unsqueeze(0)
                .numpy()
                .astype(np.float32)
            )
            output = self.session.run(None, {self.input_name: input_np})[0]
            return float(output.squeeze())
        except Exception as e:
            rospy.logerr(f"[Inference] Error: {e}")
            return None
