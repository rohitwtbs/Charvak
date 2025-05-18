from charvak_engine.core.engine import Engine
from charvak_engine.core.window import Window
from charvak_engine.graphics.renderer import Renderer
from charvak_engine.scene.scene import Scene

window = Window(800, 800, b"Charvak Physics Simulation")
renderer = Renderer()
scene = Scene(renderer.ctx)
engine = Engine(window, renderer, scene)

try:
    engine.run()
finally:
    window.terminate()
