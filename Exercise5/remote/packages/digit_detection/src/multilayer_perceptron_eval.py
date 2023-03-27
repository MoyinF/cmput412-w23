# -*- coding: utf-8 -*-
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torch.utils.data as data

from multilayer_perceptron import MLP

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import cv2

import copy
import random
import time

class DigitPredictor:
    def __init__(self, model_path, input_dim=None, output_dim=None):
        INPUT_DIM = 28 * 28 * 3
        OUTPUT_DIM = 10
        if input_dim is not None:
            INPUT_DIM = input_dim
            OUTPUT_DIM = output_dim
        self.model = MLP(INPUT_DIM, OUTPUT_DIM)
        # self.model = CNN()
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model.load_state_dict(torch.load(model_path))
        self.model = self.model.to(self.device)
        
    def normalize_input(self, input_im):
        ROOT = '.data'
        train_data = datasets.MNIST(root=ROOT,train=True,download=True)
        mean = train_data.data.float().mean() / 255
        std = train_data.data.float().std() / 255
        train_transforms = transforms.Compose([
                            transforms.RandomRotation(5, fill=(0,)),
                            transforms.RandomCrop(28, padding=2),
                            transforms.ToTensor(),
                            transforms.Normalize(mean=[mean], std=[std])
                                      ])
        self.input_transforms = transforms.Compose([ transforms.ToTensor(), transforms.Normalize(mean=[mean], std=[std])])
    
    def prepare_input(self, input_im):
        # input_im is a 28*28 cv_image. Convert to np array
        # then transform to Tensor and Normalize
        
        input_im = cv2.resize(input_im, (28, 28))
        input_im = np.expand_dims(input_im, 0)
        img_tensor = torch.from_numpy(input_im)
        
        # n = np.asarray(input_im)
        # tensor = self.input_transforms(n)
        return img_tensor.float()

    def predict(self, input_im):
        input_im = self.prepare_input(input_im) # convert from cv image to tensor
        input_im = input_im.to(self.device)
        #y_pred, _ = self.model(input)

        with torch.no_grad():
            output, _ = self.model(input_im)
            
        print("Output: " + str(output))
        pred = output.argmax().item()

        return pred

