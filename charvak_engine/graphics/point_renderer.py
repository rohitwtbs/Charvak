# charvak_engine/graphics/point_renderer.py
import numpy as np
import moderngl

class PointRenderer:
    def __init__(self, ctx, particle_count):
        self.ctx = ctx
        self.prog = ctx.program(
            vertex_shader='''
            #version 330
            in vec2 in_pos;
            void main() {
                gl_PointSize = 3.0;
                gl_Position = vec4(in_pos * 2.0 - 1.0, 0.0, 1.0);
            }
            ''',
            fragment_shader='''
            #version 330
            out vec4 fragColor;
            void main() {
                fragColor = vec4(1.0, 0.6, 0.1, 1.0);
            }
            '''
        )
        self.vbo = ctx.buffer(reserve=particle_count * 2 * 4)
        self.vao = ctx.simple_vertex_array(self.prog, self.vbo, 'in_pos')

    def update_data(self, pos_array):
        self.vbo.write(pos_array.astype('f4').tobytes())

    def render(self):
        self.vao.render(moderngl.POINTS)
