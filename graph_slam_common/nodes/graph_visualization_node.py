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

publisher = rospy.Publisher("graph_markers", MarkerArray, queue_size=10)

def graphCallback(data):
    markerArray = MarkerArray()

    nodeMarker = Marker()
    nodeMarker.header.frame_id = data.header.frame_id
    nodeMarker.id = 0
    nodeMarker.ns = "nodes"
    nodeMarker.type = nodeMarker.SPHERE_LIST
    nodeMarker.action = nodeMarker.ADD
    nodeMarker.scale.x = 0.08
    nodeMarker.scale.y = 0.08
    nodeMarker.scale.z = 0.08

    nodeMap = dict()
    num = len(data.nodes)
    step = 1 #int(math.ceil(max(1, num / 100)));
    for i in range(0, len(data.nodes), step):
        node = data.nodes[i]
        nodeMap[node.id] = node.pose
        
        nodeMarker.color.a = 1.0
        nodeMarker.color.r = 0.0
        nodeMarker.color.g = 1.0
        nodeMarker.color.b = 0.0

        nodePoint = Point()
        nodePoint.x = node.pose.position.x
        nodePoint.y = node.pose.position.y
        nodePoint.z = node.pose.position.z

        nodeMarker.points.append(nodePoint)

    markerArray.markers.append(nodeMarker)    

    edgeMarker = Marker()
    edgeMarker.id = 1
    edgeMarker.ns = "edges_valid"
    edgeMarker.header.frame_id = data.header.frame_id
    edgeMarker.type = edgeMarker.LINE_LIST
    edgeMarker.scale.x = 0.02
    edgeMarker.scale.y = 0.02
    edgeMarker.color.a = 0.5
    edgeMarker.color.r = 1.0
    edgeMarker.color.g = 0.0
    edgeMarker.color.b = 0.0
    for edge in data.edges:
        if edge.valid and edge.id_from in nodeMap and edge.id_to in nodeMap and edge.type != Edge.TYPE_2D_LASER:
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

    edgeMarker = Marker()
    edgeMarker.id = 2
    edgeMarker.ns = "edges_invalid"
    edgeMarker.header.frame_id = data.header.frame_id
    edgeMarker.type = edgeMarker.LINE_LIST
    edgeMarker.scale.x = 0.01
    edgeMarker.scale.y = 0.01
    edgeMarker.color.a = 0.3
    edgeMarker.color.r = 0.0
    edgeMarker.color.g = 0.0
    edgeMarker.color.b = 0.0
    for edge in data.edges:
        if (not edge.valid) and edge.id_from in nodeMap and edge.id_to in nodeMap and edge.type != Edge.TYPE_2D_LASER:
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

    edgeMarker = Marker()
    edgeMarker.id = 3
    edgeMarker.ns = "edges_laser_valid"
    edgeMarker.header.frame_id = data.header.frame_id
    edgeMarker.type = edgeMarker.LINE_LIST
    edgeMarker.scale.x = 0.01
    edgeMarker.scale.y = 0.01
    edgeMarker.color.a = 0.3
    edgeMarker.color.r = 1.0
    edgeMarker.color.g = 1.0
    edgeMarker.color.b = 0.0
    for edge in data.edges:
        if edge.valid and edge.id_from in nodeMap and edge.id_to in nodeMap and edge.type == Edge.TYPE_2D_LASER:
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

    # Publish the MarkerArray
    publisher.publish(markerArray)

def GraphVisualizationNode():
    rospy.init_node('graph_visualization_node', anonymous=True)
    rospy.Subscriber("display_graph", Graph, graphCallback, None, 1)
    rospy.spin()

if __name__ == '__main__':
    GraphVisualizationNode()
