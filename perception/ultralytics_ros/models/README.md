# YOLO Project Setup Guide

## 1. Installs and Setups

**FSOCO:**

Create a directory for all of your yolo projects, e.g. yolo_dir

Open cmd line, and use cd \… then mkdir yolo_dir
or can create folder in file explorer and use cd to find in cmd e.g. cd \Desktop\FSAI\yolo_dir

Install FSOCO dataset - Bounding boxes for cones, here: https://fsoco.github.io/fsoco-dataset/download

This is in Supervisely format. Supervisely is a platform meant to make training models easier, but in order to do anything useful, you need to pay.
So, instead I used a platform called Roboflow to convert the annotated images from Supervisely format to Yolov11 format.

https://roboflow.com/convert/supervisely-json-yolov11-pytorch-txt - This site should give step-by-step instructions for converting the files.

Because of the size of the dataset, I had to upload the pictures one or two folders at a time (the dataset is separated into smaller subsets), then once converted, moved the new files into a destination folder, such as FSOCO_converted, until I'd converted the entire dataset. It would be quicker to use the paid version of Roboflow, as you can upload more data at once.

Next is the python libraries.

Open up cmd prompt, and find your yolo directory with cd\… e.g. cd \Desktop\yolo_dir, and do everything from here.

For PyTorch CUDA version, you require Python 3.9-3.12 (Maybe 3.13) (run in cmd line: python --version)

Ensure to install everything while working in the directory containing the FSOCO dataset.

https://aleksandarhaber.com/install-and-run-you-only-look-once-yolo-computer-vision-model-on-gpu-cpu-and-windows/ - Has the steps I've outlined below


Optional command in yolo_dir (create python environment, I didn't need to):

```bash
python -m venv env1
env1\Scripts\activate.bat ### This activates the environment, instead of using conda env or something
```

Important commands in yolo_dir:

```bash
pip install setuptools
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu124
pip install git+https://github.com/ultralytics/ultralytics.git@main
```

After this, I installed a pre-trained Yolo11 model, for varied object detection, to be the basic model to be trained. You can also use an entirely untrained model. To download one of the pre-trained models, go to https://github.com/ultralytics/ultralytics, scroll to the table of models, and select e.g. Yolo11n, which is what I used. Put this in the yolo directory.

Now, you should be ready to run some yolo commands. 


## 2. Testing yolo with pre-trained model


Before training the model, you can test it out on a random stock image:

```bash
yolo detect predict model=yolo11n.pt source='path to source e.g. Desktop/example.jpg'
```

I simply downloaded a stock image of a horse, and moved it into yolo_dir, before running this command. This should create a runs folder, and you will find the labelled image in runs/detect/predict.


## 3. Test with last year's model (Optional):

**IMPORTANT NOTE:** This year's model does work. I downloaded it straight from the gryphonracingAI GitHub, and ran the same command on a picture of cones, using model=conev11n.pt. As a failsafe, you can always use that model for testing etc.


## 4. Training:

In order to train a model, the main thing you will need is a .yaml file. This is used to tell yolo where the data is. This looks like this:

```yaml
# Dataset root directory
path: ../datasets/coco8 # Root directory for the dataset

# Paths to training, validation, and testing images
train: images/train # Training images (relative to 'path')
val: images/val # Validation images (relative to 'path')
test: images/test # Testing images (optional)

# Number of classes and their names
nc: 5 # Number of classes
names: # Class names
  0: blue_cone
  1: large_orange_cone
  2: orange_cone
  3: unknown_cone
  4: yellow_cone
```

To create the yaml, first, create data.txt in the yolo_dir directory, paste the above text, change the directories to your own, then save and change the file extension to .yaml (YOU CAN'T EDIT IT AFTER THIS).

To start, I would pick a subset of the data, to train on, so it takes less time and you can make sure that it works.


**IMPORTANT:** I can't remember exactly the format of the folders in FSOCO, so you might need to play around with it.

Here is the training command:

```bash
yolo detect train data=data.yaml model=yolov8n.pt epochs=100 imgsz=64
```

data = your yaml file, model = the model you'd like to train (Perhaps use my model as a base, or an empty model), epochs=number of runs through the dataset (start with small numbers e.g. 5,10), imgz= I don't really know what's best, but 64 should work fine.


## 5. Results

The results of training should be stored in runs/train/, where the best model (best weights) are stored in runs/train/exp/weights/best.pt. 'best.pt' is the new model, which you can rename and use in more training if you'd like.

There are ways to configure the results of training such that it gives you useful matrices and such to compare with this year. I'll leave that to you, as I can't remember the ins and outs anymore. If you have any questions, feel free to message me! My number is 07737 983 861.

I hope this works for you! I ran through the first 3 steps myself while making this tutorial, so it should work, but the training part I am not able to test myself.
