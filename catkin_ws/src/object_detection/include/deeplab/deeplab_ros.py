#/usr/bin/env python3

# Copyright 2021 ROS-Force
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
# and associated documentation files (the "Software"), to deal in the Software without restriction, 
# including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
# and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, 
# subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all copies 
# or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED 
# TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
# ==============================================================================

import numpy as np
import cv2
import tensorflow as tf
import include.get_dataset_colormap as dlutils

class DeepLabModel(object):
  """Class to load DeepLab model and run inference on supplied images."""

  def __init__(self, modelConfig):
    """Creates and loads pretrained DeepLab model."""

    # grab model config
    self.modelConfig = modelConfig
 
    with open(self.modelConfig.graphPath, 'rb') as file_handle:
      # load frozen inference graph from file and wrap it into a tf.WrapperFunction
      self.modelFunction = DeepLabModel.__wrap_frozen_graph(tf.compat.v1.GraphDef.FromString(file_handle.read()), self.modelConfig.inputTensorName, self.modelConfig.outputTensorName)

  def run(self, image, isImageRGB=True):
    """Runs inference on a single image.

    Args:
      image: A OpenCV RGB image.

    Returns:
      result: Segmentation map of the input image.
          
    TODO: Adjust model input & output according to the supplied param (modelConfig.outputTensorName & modelConfig.inputTensorName)
    """ 
    
    # resize image to fit model's input
    input_image = cv2.resize(image, self.modelConfig.inputSize, interpolation=cv2.INTER_CUBIC)
    
    # change to BGR if model was trained with BGR images and the image is in RGB
    if (self.inputBGR and isImageRGB):
      input_image = cv2.cvtColor(input_image, cv2.COLOR_RGB2BGR)

    elif (~self.inputBGR and ~isImageRGB):
      input_image = cv2.cvtColor(input_image, cv2.COLOR_BGR2RGB)
      
    # add extra dimension and convert array to tf.Tensor (or more accurately, to an ImageTensor)
    segmentation_map = tf.squeeze(self.modelFunction(tf.expand_dims(tf.convert_to_tensor(input_image), axis=0)))

    # convert map to np array and return reshaped segmentation map to fit original image
    return cv2.resize(segmentation_map.numpy().astype('uint8'), image.shape[1::-1], interpolation=cv2.INTER_NEAREST)
  
  def getColormapFromSegmentationMap(self, segmentation_map):
    """Adds color defined by the dataset colormap to the label.

    Args:
      segmentation_map: A 2D array with integer type, storing the segmentation label.

    Returns:
      result: A 2D array with floating type. The element of the array
        is the color indexed by the corresponding element in the input label
        to the dataset color map.

    """

    return dlutils.label_to_color_image(segmentation_map, self.modelConfig.datasetName)
    
  def __wrap_frozen_graph(graph_def, inputs, outputs):
    """ 
    Wrapper for frozen inference graph (works as a compatibility layer between TF 1.x and TF 2.x). 
    
    Args:
      graph_def: Represents the graph of operations.
      inputs: Text description of the input tensors of the graph.
      outputs: Text description of the input tensors of the graph.

    Returns:
      result: tf.WrappedFunction of the imported graph.
      
    """

    wrapped_import = tf.compat.v1.wrap_function(lambda: tf.compat.v1.import_graph_def(graph_def, name=""), [])
    import_graph = wrapped_import.graph
    return wrapped_import.prune(
        tf.nest.map_structure(import_graph.as_graph_element, inputs),
        tf.nest.map_structure(import_graph.as_graph_element, outputs))

class DeepLabModelConfig(object):
  '''
  Helper class to hold the DeepLab model's parameters. 
  '''

  def __init(self, graphPath, datasetName, inputTensorName, inputBGR, outputTensorName, inputSize):
    self.graphPath = graphPath
    self.datasetName = datasetName
    self.inputBGR = inputBGR
    self.inputTensorName = inputTensorName
    self.outputTensorName = outputTensorName
    self.inputSize = np.array(inputSize)
    