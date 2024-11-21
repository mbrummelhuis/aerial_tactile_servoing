"""
TacTip driver for ROS2, heavily inspired by RealSensor implementation
"""
import os
import cv2

from dependencies.image_transforms import process_image
from dependencies.models import create_model
from dependencies.utils import load_json_obj
from dependencies.label_encoder import LabelEncoder
from dependencies.label_encoder import BASE_MODEL_PATH 
from dependencies.labelled_model import LabelledModel

class TacTip:
    def __init__(self):
        # set up the camera
        self.source = 4
        self.cam = cv2. VideoCapture(self.source)
        for _ in range(10):
            self.cam.read()
        
        # initialize params
        self.model_label_params = {}
        self.model_image_params = {}
        self.model_params = {}
        self.setup_params()

        # create the label encoder/decoder
        label_encoder = LabelEncoder(self.model_label_params, device='cuda')
        
        # setup the model
        model = create_model(
            in_dim=self.model_image_params['image_processing']['dims'],
            in_channels=1,
            out_dim=label_encoder.out_dim,
            model_params=self.model_params,
            saved_model_dir=self.model_dir,
            device='cuda'
        )
        model.eval()

        self.pose_model = LabelledModel(
            model,
            self.model_image_params['image_processing'],
            label_encoder,
            device='cuda'
        )

    def setup_params(self):
        self.model_label_params = load_json_obj(os.path.join(BASE_MODEL_PATH, 'model_label_params'))
        self.mode_image_params = load_json_obj(os.path.join(BASE_MODEL_PATH, 'model_image_params'))
        self.model_params = load_json_obj(os.path.join(BASE_MODEL_PATH, 'model_params'))

    def read(self):
        _, img = self.cam.read()
        return img

    def process(self, outfile=None):
        img = self.read()
        img = process_image(img, **self.sensor_params)
        if outfile:
            cv2.imwrite(outfile, img)
        return img
    
    def predict(self, processed_img):
        return self.pose_model.predict(processed_img)

    def get_measurement(self):
        img = self.process()
        return self.predict(img)