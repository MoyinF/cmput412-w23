# -*- coding: utf-8 -*-
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torch.utils.data as data
from torch.utils.data import Dataset, DataLoader

from multilayer_perceptron import MLP
from convolutional_nn import CNN

import matplotlib.pyplot as plt
import numpy as np
import glob
import cv2

import copy
import random
import time

SEED = 1234

random.seed(SEED)
np.random.seed(SEED)
torch.manual_seed(SEED)
torch.cuda.manual_seed(SEED)
torch.backends.cudnn.deterministic = True

DEBUG = False
model_is_mlp = False

def blue_mask(image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # blue colors
        lower_range = np.array([86, 66, 95])
        upper_range = np.array([179,255,255])
        mask = cv2.inRange(hsv, lower_range, upper_range)
        # With canny edge detection:
        edged = cv2.Canny(mask, 30, 200)

        return mask

def mask_img(img):
    # take in cv_image # if img is filename: im = cv2.imread(f'{img}')
    im = blue_mask(img)

    # add line(s) around border to prevent floodfilling the digits
    # cv2.line(im, (0,28), (28,28), (255, 255, 255), 1)
    # cv2.line(im, (0,self.INPUT_W), (self.INPUT_H,self.INPUT_W), (255, 255, 255), 1)
    # cv2.line(im, (0,self.INPUT_W), (self.INPUT_H,self.INPUT_W), (255, 255, 255), 1)
    # cv2.line(im, (0,self.INPUT_W), (self.INPUT_H,self.INPUT_W), (255, 255, 255), 1)
    global DEBUG
    if DEBUG:
        cv2.imshow("image", im)
        cv2.waitKey(1000)
        cv2.destroyWindow("image")

    # im = cv2.copyMakeBorder(im, 1, 1, 1, 1, cv2.BORDER_CONSTANT, None, value = 0)
    # cv2.floodFill(im, None, (0, 28), 255)
    # cv2.floodFill(im, None, (28, 0), 255)
    # cv2.floodFill(im, None, (0, 0), 255)
    # cv2.floodFill(im, None, (28, 28), 255)
    im = cv2.resize(im, (28, 28))
    # im = cv2.bitwise_not(im)
    return im

class CustomDataset(Dataset):
    def __init__(self):
        self.imgs_path = "test_images/"
        file_list = glob.glob(self.imgs_path + "*")
        # print(file_list)

        self.data = []
        for class_path in file_list:
            class_name = class_path.split("/")[-1]
            for img_path in glob.glob(class_path + "/*.jpg"):
                self.data.append([img_path, class_name])
        # print(self.data)

        self.class_map = {
            "0": 0,
            "1": 1,
            "2": 2,
            "3": 3,
            "4": 4,
            "5": 5,
            "6": 6,
            "7": 7,
            "8": 8,
            "9": 9
        }

        self.img_dim = (28, 28)

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        img_path, class_name = self.data[idx]
        # same steps as in prepare_input in eval.py
        img = cv2.imread(img_path)
        img = cv2.resize(img, self.img_dim)
        # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = mask_img(img)
        img_tensor = torch.from_numpy(img)

        class_id = self.class_map[class_name]
        class_id = torch.tensor(class_id)

        return img_tensor.float(), class_id

def calculate_accuracy(y_pred, y):
    top_pred = y_pred.argmax(1, keepdim=True)
    correct = top_pred.eq(y.view_as(top_pred)).sum()
    acc = correct.float() / y.shape[0]
    return acc

def train(model, iterator, optimizer, criterion, device):

    epoch_loss = 0
    epoch_acc = 0

    model.train()

    for (x, y) in iterator:

        x = x.to(device)
        y = y.to(device)

        optimizer.zero_grad()

        if model_is_mlp:
            y_pred, _ = model(x) # for MLP
        else:
            y_pred = model(x) # for CNN

        loss = criterion(y_pred, y)

        acc = calculate_accuracy(y_pred, y)

        loss.backward()

        optimizer.step()

        epoch_loss += loss.item()
        epoch_acc += acc.item()

    return epoch_loss / len(iterator), epoch_acc / len(iterator)

def evaluate(model, iterator, criterion, device):

    epoch_loss = 0
    epoch_acc = 0

    model.eval()

    with torch.no_grad():

        for (x, y) in iterator:

            x = x.to(device)
            y = y.to(device)

            if model_is_mlp:
                y_pred, _ = model(x) # for MLP
            else:
                y_pred = model(x) # for CNN

            loss = criterion(y_pred, y)

            acc = calculate_accuracy(y_pred, y)

            epoch_loss += loss.item()
            epoch_acc += acc.item()

    return epoch_loss / len(iterator), epoch_acc / len(iterator)


def epoch_time(start_time, end_time):
    elapsed_time = end_time - start_time
    elapsed_mins = int(elapsed_time / 60)
    elapsed_secs = int(elapsed_time - (elapsed_mins * 60))
    return elapsed_mins, elapsed_secs


dataset = CustomDataset()

TRAIN_RATIO = 0.7
VALID_RATIO = 0.5

n_train_examples = int(len(dataset) * TRAIN_RATIO)
n_valid_examples = int((len(dataset) - n_train_examples) * VALID_RATIO)
n_test_examples = int(len(dataset) - n_train_examples - n_valid_examples)

train_data, valid_data, test_data = data.random_split(dataset, [n_train_examples, n_valid_examples, n_test_examples])

print(f'Number of training examples: {len(train_data)}')
print(f'Number of validation examples: {len(valid_data)}')
print(f'Number of testing examples: {len(test_data)}')

BATCH_SIZE = 1 # for CNN

if model_is_mlp:
    BATCH_SIZE = 64 # for MLP
else:
    BATCH_SIZE = 1 # for CNN

train_iterator = data.DataLoader(train_data,
                                 shuffle=True,
                                 batch_size=BATCH_SIZE)

valid_iterator = data.DataLoader(valid_data,
                                 batch_size=BATCH_SIZE)

test_iterator = data.DataLoader(test_data,
                                batch_size=BATCH_SIZE)



INPUT_DIM = 28 * 28
OUTPUT_DIM = 10

model = None
if model_is_mlp:
    model = MLP(INPUT_DIM, OUTPUT_DIM)
else:
    model = CNN()

optimizer = optim.Adam(model.parameters())

criterion = nn.CrossEntropyLoss()

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

model = model.to(device)
criterion = criterion.to(device)


EPOCHS = 10

best_valid_loss = float('inf')

for epoch in range(EPOCHS):

    start_time = time.monotonic()

    train_loss, train_acc = train(model, train_iterator, optimizer, criterion, device)
    valid_loss, valid_acc = evaluate(model, valid_iterator, criterion, device)

    if valid_loss < best_valid_loss:
        best_valid_loss = valid_loss
        torch.save(model.state_dict(), 'ex5-model.pt')

    end_time = time.monotonic()

    epoch_mins, epoch_secs = epoch_time(start_time, end_time)

    print(f'Epoch: {epoch+1:02} | Epoch Time: {epoch_mins}m {epoch_secs}s')
    print(f'\tTrain Loss: {train_loss:.3f} | Train Acc: {train_acc*100:.2f}%')
    print(f'\t Val. Loss: {valid_loss:.3f} |  Val. Acc: {valid_acc*100:.2f}%')


model.load_state_dict(torch.load('ex5-model.pt'))
test_loss, test_acc = evaluate(model, test_iterator, criterion, device)
print(f'Test Loss: {test_loss:.3f} | Test Acc: {test_acc*100:.2f}%')
