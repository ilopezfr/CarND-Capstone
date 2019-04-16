from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import rospy
import math
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
#from PIL import Image as PILImage # debugging if you want to save the images

class TLClassifier(object):
    def __init__(self):
        # TODO load classifier
        if tf.__version__ < '1.4.0':
            rospy.logwarn('Please upgrade your tensorflow installation to v1.4.* or later! you have version '+tf.__version__ )
            rospy.logwarn('Ignore this message if you run an older tensorflow version intentionally.')
        PATH_TO_CKPT = 'light_classification/model/frozen_inference_graph.pb'
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        self.bounding_box_img_pubs = rospy.Publisher('/debug/bounding_box_img', Image, queue_size=1)
        self.bridge = CvBridge()
        #self.imagenumber = 0

    def state_to_string(self, state):
        """ Returns the color light associated with the state """
        if (state == 1 ):
            return "GREEN"
        elif (state == 2 ):
            return "YELLOW"
        elif (state == 3 ):
            return "RED"
        else:
            return "UNKNOWN"

    def add_bounding_box_to_image(self, image, box):
        img_height = image.shape[0] # height 600
        img_width = image.shape[1]  # width 800
        y_min, x_min, y_max, x_max = box
        box_width = (x_max - x_min) * img_width
        box_height = (y_max - y_min) * img_height
        top_left = int(x_min * img_width), int(y_min * img_height)
        bot_right = int(x_max * img_width), int(y_max * img_height)
        print("top-left: " + str(top_left) + " bot_right " + str(bot_right))
        cv2.rectangle(image, top_left, bot_right, (0, 255, 0), 10)
        return image

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        
        # TODO implement light color prediction
        TL_PROB_THRESHOLD = 0.35
        image_np = self.load_image_into_numpy_array(image)
        # im = PILImage.fromarray(image_np) # debugging if you want to save the images
        # im.save(str(self.imagenumber)+"_result.jpg", "JPEG", quality=80, optimize=True, progressive=True) # debugging if you want to save the images
        # self.imagenumber = self.imagenumber +1

        # Actual detection.
        output_dict = self.run_inference_for_single_image(image_np, self.detection_graph)
        text_string = "Classified light (idx {0}) state : {1} with probability {2}"
        if (output_dict['detection_scores'][0] > TL_PROB_THRESHOLD):
            text_string += " > " + str(TL_PROB_THRESHOLD)
        else:
            text_string += " <= " + str(TL_PROB_THRESHOLD)
        max_box_idx = -1
        max_box_size = -1.0
        for i in range(output_dict['num_detections']/2):
            if (output_dict['detection_scores'][i] > TL_PROB_THRESHOLD):
                x = output_dict['detection_boxes'][i][3] - output_dict['detection_boxes'][i][1]
                y = output_dict['detection_boxes'][i][2] - output_dict['detection_boxes'][i][0]
                box_size = math.sqrt((x * x) + (y * y))
                print("i: " + str(i) + " score: " + str(output_dict['detection_scores'][i]) + " " + self.state_to_string(output_dict['detection_classes'][i]) + " box_size: " + str(box_size) + " " + str(output_dict['detection_boxes'][i]))
                # print(output_dict['detection_boxes'][i])
                if (box_size > max_box_size):
                    max_box_idx = i
                    max_box_size = box_size

        ret_val = TrafficLight.UNKNOWN
        if ((max_box_idx >= 0) and (output_dict['detection_scores'][max_box_idx] > TL_PROB_THRESHOLD)):
            # rospy.logwarn(text_string.format(max_box_idx, output_dict['detection_classes'][max_box_idx], output_dict['detection_scores'][max_box_idx]))
            print(text_string.format(max_box_idx, output_dict['detection_classes'][max_box_idx], output_dict['detection_scores'][max_box_idx]))
            image = self.add_bounding_box_to_image(image, output_dict['detection_boxes'][max_box_idx])

            # New model assignment
            if (output_dict['detection_classes'][max_box_idx] == 1 ):
                ret_val = TrafficLight.GREEN
            elif (output_dict['detection_classes'][max_box_idx] == 2 ):
                ret_val = TrafficLight.YELLOW
            elif (output_dict['detection_classes'][max_box_idx] == 3 ):
                ret_val = TrafficLight.RED

        # publish debug bounding box image
        img_msg = self.bridge.cv2_to_imgmsg(image, encoding="rgb8")
        self.bounding_box_img_pubs.publish(img_msg)

        return ret_val

    def load_image_into_numpy_array(self,image):
        im_height, im_width = image.shape[:2]
        #(im_width, im_height) = image.size
        return image.reshape((im_height, im_width, 3)).astype(np.uint8)

    def run_inference_for_single_image(self,image, graph):
        with graph.as_default():
            with tf.Session() as sess:
                # Get handles to input and output tensors
                ops = tf.get_default_graph().get_operations()
                all_tensor_names = {output.name for op in ops for output in op.outputs}
                tensor_dict = {}
                for key in [
                    'num_detections', 'detection_boxes', 'detection_scores',
                    'detection_classes', 'detection_masks'
                ]:
                    tensor_name = key + ':0'
                    if tensor_name in all_tensor_names:
                        tensor_dict[key] = tf.get_default_graph().get_tensor_by_name(tensor_name)

                image_tensor = tf.get_default_graph().get_tensor_by_name('image_tensor:0')

                # Run inference
                output_dict = sess.run(tensor_dict,
                                        feed_dict={image_tensor: np.expand_dims(image, 0)})

                # all outputs are float32 numpy arrays, so convert types as appropriate
                output_dict['num_detections'] = int(output_dict['num_detections'][0])
                output_dict['detection_classes'] = output_dict[
                    'detection_classes'][0].astype(np.uint8)
                output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
                output_dict['detection_scores'] = output_dict['detection_scores'][0]

        return output_dict
