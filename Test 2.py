#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Dec 11 18:53:29 2017

@author: Chaitanya
"""
import math
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def distance(lat1, long1, lat2, long2):
    # Note: The formula used in this function is not exact, as it assumes
    # the Earth is a perfect sphere.

    # Mean radius of Earth in miles
    radius_earth = 3959

    # Convert latitude and longitude to
    # spherical coordinates in radians.
    degrees_to_radians = math.pi/180.0
    phi1 = lat1 * degrees_to_radians
    phi2 = lat2 * degrees_to_radians
    lambda1 = long1 * degrees_to_radians
    lambda2 = long2 * degrees_to_radians
    dphi = phi2 - phi1
    dlambda = lambda2 - lambda1

    a = haversine(dphi) + math.cos(phi1) * math.cos(phi2) * haversine(dlambda)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = radius_earth * c
    return d

def haversine(angle):
  h = math.sin(angle / 2) ** 2
  return h
# Distance callback

class CreateDistanceCallback(object):
  """Create callback to calculate distances between points."""

  def __init__(self):

    # Latitudes and longitudes of selected U.S. cities

    locations = [[40.489806,-78.403319],
                 [40.808499,-77.895752],
                 [40.803390,-77.883049],
                 [40.792134,-77.867813],
                 [40.800097,-77.867198],
                 [40.831305,-77.843047],
                 [40.823346,-77.874715],
                 [40.814783,-77.854642],
                 [40.808623,-77.855536],
                 [40.807439,-77.856156],
                 [40.803221,-77.861796],
                 [40.803845,-77.865218],
                 [40.797601,-77.866382],
                 [40.793705,-77.868122],
                 [40.792065,-77.863437],
                 [40.793436,-77.860714],
                 [40.793360,-77.859769],
                 [40.798125,-77.855862],
                 [40.798464,-77.855829],
                 [40.794870,-77.860170],
                 [40.832941,-77.800758]]

    """Create the distance matrix."""
    size = len(locations)
    self.matrix = {}

    for from_node in range(size):
      self.matrix[from_node] = {}
      for to_node in range(size):
        if from_node == to_node:
          self.matrix[from_node][to_node] = 0
        else:
          x1 = locations[from_node][0]
          y1 = locations[from_node][1]
          x2 = locations[to_node][0]
          y2 = locations[to_node][1]
          self.matrix[from_node][to_node] = distance(x1, y1, x2, y2)

  def Distance(self, from_node, to_node):
    return int(self.matrix[from_node][to_node])

def main():


  # Cities
  city_names = ["New York", "Los Angeles", "Chicago", "Minneapolis", "Denver", "Dallas", "Seattle",
                "Boston", "San Francisco", "St. Louis","x","y","z","a","b","c","d","e","f","g","h","i","j","k","l","m","n","o","p","q","r","s","t","w","aa","bb","cc","dd"]
  tsp_size = len(city_names)
  num_routes = 1    # The number of routes, which is 1 in the TSP.
  # Nodes are indexed from 0 to tsp_size - 1. The depot is the starting node of the route.
  depot = 0

  # Create routing model
  if tsp_size > 0:
    routing = pywrapcp.RoutingModel(20, 1, 0)
    search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()

    # Create the distance callback, which takes two arguments (the from and to node indices)
    # and returns the distance between these nodes.
    dist_between_nodes = CreateDistanceCallback()
    dist_callback = dist_between_nodes.Distance
    routing.SetArcCostEvaluatorOfAllVehicles(dist_callback)
    # Solve, returns a solution if any.
    assignment = routing.SolveWithParameters(search_parameters)
    if assignment:
      # Solution cost.
      print ("Total distance: " + str(assignment.ObjectiveValue()) + " miles\n")
      # Inspect solution.
      # Only one route here; otherwise iterate from 0 to routing.vehicles() - 1
      route_number = 0
      index = routing.Start(route_number) # Index of the variable for the starting node.
      route = ''
      while not routing.IsEnd(index):
        # Convert variable indices to node indices in the displayed route.
        route += str(city_names[routing.IndexToNode(index)]) + ' -> '
        index = assignment.Value(routing.NextVar(index))
      route += str(city_names[routing.IndexToNode(index)])
      print ("Route:\n\n" + route)
    else:
      print ('No solution found.')
  else:
    print ('Specify an instance greater than 0.')

if __name__ == '__main__':
  main()