# Load "env.obj" and "rob.obj" in gepetto-gui

import os

from gepetto.corbaserver import Client

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

q2 = (0, 0, 0, 0, 0, 0, 1)
q1 = (-1435.35587657243, 2891.398094594479, 1462.830701842904) + (
    0.6741912139367736,
    -0.2384437590607974,
    0.6418622372743962,
    -0.2768097707389008,
)

c.gui.applyConfiguration("env", q1)
c.gui.applyConfiguration("rob", q2)

c.gui.refresh()
