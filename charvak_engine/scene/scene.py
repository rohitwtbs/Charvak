# charvak_engine/scene/scene.py
from charvak_engine.compute.physics import PhysicsSimulation
from charvak_engine.graphics.point_renderer import PointRenderer

class Scene:
    def __init__(self, ctx):
        self.sim = PhysicsSimulation()
        self.renderer = PointRenderer(ctx, 4096)

    def update(self, dt):
        self.sim.step(dt)
        pos = self.sim.get_positions()
        self.renderer.update_data(pos)

    def draw(self, ctx):
        self.renderer.render()
