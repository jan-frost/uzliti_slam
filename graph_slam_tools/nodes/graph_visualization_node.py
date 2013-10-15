#!/usr/bin/env python

import roslib; roslib.load_manifest('graph_slam_tools')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from graph_slam_msgs.msg import Graph
from graph_slam_msgs.msg import Node
from graph_slam_msgs.msg import Edge
from geometry_msgs.msg import Point
import rospy
import tf
import numpy
import math

publisher = rospy.Publisher("graph_markers", MarkerArray, queue_size=5)

def graphCallback(data):
    markerArray = MarkerArray()

    nodeMarker = Marker()
    nodeMarker.header.frame_id = data.header.frame_id
    nodeMarker.id = 0
    nodeMarker.type = nodeMarker.SPHERE_LIST
    nodeMarker.action = nodeMarker.ADD
    nodeMarker.scale.x = 0.15
    nodeMarker.scale.y = 0.15
    nodeMarker.scale.z = 0.15

    nodeMap = dict()
    num = len(data.nodes)
    step = 1 #int(math.ceil(max(1, num / 100)));
    for i in range(0, len(data.nodes), step):
        node = data.nodes[i]
        nodeMap[node.id] = node.pose
        
        if node.is_loop:
            nodeMarker.color.a = 1.0
            nodeMarker.color.r = 0.0
            nodeMarker.color.g = 1.0
            nodeMarker.color.b = 0.0
        elif node.is_border:
            nodeMarker.color.a = 1.0
            nodeMarker.color.r = 0.0
            nodeMarker.color.g = 1.0
            nodeMarker.color.b = 1.0
        elif node.is_in_scope:
            nodeMarker.color.a = 1.0
            nodeMarker.color.r = 0.0
            nodeMarker.color.g = 0.0
            nodeMarker.color.b = 1.0
        else:
            nodeMarker.color.a = 1.0
            nodeMarker.color.r = 0.0
            nodeMarker.color.g = 0.0
            nodeMarker.color.b = 0.0

        nodePoint = Point()
        nodePoint.x = node.pose.position.x
        nodePoint.y = node.pose.position.y
        nodePoint.z = node.pose.position.z

        nodeMarker.points.append(nodePoint)
    
    

    edgeMarker = Marker()
    edgeMarker.id = 1
    edgeMarker.header.frame_id = data.header.frame_id
    edgeMarker.type = edgeMarker.LINE_LIST
    edgeMarker.scale.x = 0.08
    edgeMarker.scale.y = 0.08
    edgeMarker.color.a = 0.5
    edgeMarker.color.r = 0.0
    edgeMarker.color.g = 0.0
    edgeMarker.color.b = 0.0
    for edge in data.edges:
        if True: #edge.id_from in nodeMap and edge.id_to in nodeMap:
            pointFrom = Point()
            pointFrom.x = nodeMap[edge.id_from].position.x
            pointFrom.y = nodeMap[edge.id_from].position.y
            pointFrom.z = nodeMap[edge.id_from].position.z
        
            pointTo = Point()
            pointTo.x = nodeMap[edge.id_to].position.x
            pointTo.y = nodeMap[edge.id_to].position.y
            pointTo.z = nodeMap[edge.id_to].position.z
            
            edgeMarker.points.append(pointFrom)
            edgeMarker.points.append(pointTo)

    markerArray.markers.append(edgeMarker)
    markerArray.markers.append(nodeMarker)

    # Publish the MarkerArray
    publisher.publish(markerArray)

def GraphVisualizationNode():
    rospy.init_node('graph_visualization_node', anonymous=True)
    rospy.Subscriber("display_graph", Graph, graphCallback, None, 1)
    rospy.spin()

if __name__ == '__main__':
    GraphVisualizationNode()
