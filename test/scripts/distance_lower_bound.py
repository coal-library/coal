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
c.gui.addToGroup("env", sceneName)
c.gui.addBox("box", 500, 200, 150, Blue)
c.gui.addToGroup("box", sceneName)

q2 = (0, 0, 0, 0, 0, 0, 1)
q1 = (
    608.56046341359615,
    1624.1152798756957,
    2661.5910432301462,
    0.8083991299501978,
    0.25803832576728564,
    0.47026407332553366,
    0.24240208429437343,
)

c.gui.applyConfiguration("env", q1)
c.gui.applyConfiguration("rob", q2)

c.gui.refresh()
