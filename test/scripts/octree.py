# Load "env.obj" and "rob.obj" in gepetto-gui

import csv
import os

from gepetto.corbaserver import Client

pos = list()
with open("/home/florent/devel/hpp/src/hpp-fcl/build-rel/test/rob.octree", "r") as f:
    r = csv.reader(f, delimiter=",")
    for line in r:
        pos.append(map(float, line))

path = None
devel_hpp_dir = os.getenv("DEVEL_HPP_DIR")
if devel_hpp_dir:
    path = devel_hpp_dir + "/src/hpp-fcl/test/fcl_resources"
else:
    path = os.getenv("PWD") + "/fcl_resources"

Red = [1, 0, 0, 0.5]
Green = [0, 1, 0, 0.5]
Blue = [0, 0, 1, 0.5]

c = Client()
wid = 0

sceneName = "scene"
wid = c.gui.createWindow("test-fcl")

c.gui.createScene(sceneName)
c.gui.addSceneToWindow(sceneName, wid)

c.gui.addMesh("env", path + "/env.obj")
c.gui.addMesh("rob", path + "/rob.obj")
c.gui.addToGroup("env", sceneName)
c.gui.addToGroup("rob", sceneName)

# closest points
c.gui.addSphere("p1", 10, Red)
c.gui.addToGroup("p1", sceneName)


q1 = (-1373.283643275499, -396.2224237620831, 259.5808934420347) + (
    0.5956794918568784,
    -0.2147188057951074,
    0.257436335996247,
    0.7299234962157893,
)
q2 = (0, 0, 0, 0, 0, 0, 1)

c.gui.applyConfiguration("rob", q1)
c.gui.applyConfiguration("env", q2)

p1 = (-1990.2983245164919, -105.42114741459312, 359.74684390031132, 0, 0, 0, 1)

c.gui.applyConfiguration("p1", p1)
c.gui.refresh()
