# charvak_engine/compute/physics.py
import taichi as ti

ti.init(arch=ti.gpu)

n = 4096
positions = ti.Vector.field(2, ti.f32, shape=n)
velocities = ti.Vector.field(2, ti.f32, shape=n)
gravity_center = ti.Vector([0.5, 0.5])
gravity_strength = 0.5

@ti.kernel
def initialize():
    for i in range(n):
        positions[i] = ti.Vector([ti.random(), ti.random()])
        velocities[i] = ti.Vector([0.0, 0.0])

@ti.kernel
def update(dt: ti.f32):
    for i in range(n):
        dir = gravity_center - positions[i]
        dist = dir.norm() + 1e-3
        force = gravity_strength * dir.normalized() / dist
        velocities[i] += force * dt
        positions[i] += velocities[i] * dt

class PhysicsSimulation:
    def __init__(self):
        initialize()

    def step(self, dt):
        update(dt)

    def get_positions(self):
        return positions.to_numpy()
