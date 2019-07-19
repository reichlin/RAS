#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from keras.models import model_from_json
from keras.models import Model
import tensorflow as tf
from keras import backend as K
from ras_msgs.msg import classified_object
#import matplotlib.pyplot as plt

class NN:

    # global graph

    def __init__(self):

        rospy.init_node('classifier')


        # managing topics
        rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.readFrame, queue_size=1) # frame from the camera
        rospy.Subscriber("/classify", Point, self.classify, queue_size=1) # classify topic
        
        self.bridge = CvBridge()
        
        self.pub = rospy.Publisher("/object/classification", classified_object, queue_size=1) # response topic
        
        # variables for synchronizations
        #self.storedFrame = False
        #self.doPrediction = False
        
        #load the model
        json_file = open('/home/ras28/catkin_ws/src/ras_project/ras_detection/model.json', 'r')
        loaded_model_json = json_file.read()
        json_file.close()
        self.model = model_from_json(loaded_model_json)
        
        # load weights into new model
        self.model.load_weights("/home/ras28/catkin_ws/src/ras_project/ras_detection/model.h5")
        self.graph = tf.get_default_graph()
        self.model._make_predict_function()
        print("Loaded model from disk")
        
        # define dictionary between labels and name of objects
        self.objects = {0: "blue Cube",
                        1: "blue Triangle",
                        2: "green Cube",
                        3: "green Cylinder",
                        4: "green Hollow Cube",
                        5: "orange Cross",
                        6: "patric",
                        7: "purple Cross",
                        8: "purple Star",
                        9: "red Ball",
                       10: "red Cube",
                       11: "red Cylinder",
                       12: "yellow Ball",
                       13: "yellow Cube"}
                       

        rate = rospy.Rate(10) # 10hz
        
        
            
    def listen(self):
    
        rospy.spin()
    

        
    def readFrame(self, data):
        #print "im in read frame"
        #if self.doPrediction:
        
        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
        except CvBridgeError as e:
          print(e)
        
        self.cv_image = cv_image / 255.0 # cv2.resize(cv_image, (224, 224)) / 255.0
        #self.x = np.asarray(cv_image)
        #self.x = np.expand_dims(self.x, axis=0)
        
        #self.storedFrame = True
        #self.doPrediction = False
            
            
    def classify(self, data):
        #print "im in classify"
        #if self.storedFrame:


        top_x = int(data.x) + 100
        bottom_x = int(data.x) - 100
        top_y = int(data.y) + 100
        bottom_y = int(data.y) - 100
        
        if top_x >= 640:
            top_x = 639
        if bottom_x < 0:
            bottom_x = 0
        if top_y >= 480:
            top_y = 479
        if bottom_y < 0:
            bottom_y = 0
            
            
        image = cv2.resize(self.cv_image[bottom_y:top_y, bottom_x:top_x], (200, 200))
        
        #plt.imshow(image)
        #plt.show()
        
        print("from " + str(bottom_x) + " and " + str(bottom_y) + " to " + str(top_x) + " and " + str(top_y))
        
        
        #plt.imshow(image)
        #plt.show()
        
        x = np.asarray(image)
        x = np.expand_dims(x, axis=0)
        
        

        
        with self.graph.as_default():
            prediction = self.model.predict(x, batch_size=1, verbose=0, steps=None)
        
        
        winner = int(np.argmax(prediction, axis=-1))
        probability = prediction[0, winner]
        
        # put in a message objects[winner] and probability

        msg = classified_object()
        msg.p = probability
       # msg.name = str(self.objects[winner])
        msg.name = winner
        
        self.pub.publish(msg)
        
        print (str(self.objects[winner]) + " p: " + str(probability))
            
            
            #self.storedFrame = False
            
        #else:
            #self.doPrediction = True


                



if __name__ == '__main__':
    nn = NN()
    nn.listen()




    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
