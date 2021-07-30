import sympy as sp
import marshal
import numpy as np
import types
from numpy import array, cos, sin, sqrt
from sympy.algebras.quaternion import Quaternion


def load_sympy_func(path):
    with open(path, "rb") as f:
        code = marshal.load(f)
    return types.FunctionType(code, globals(), "some_func_name")



def sym_to_numpy(symbols, symb_expr, file_name):
    # convert symbolic expression to numpy function
    func = sp.lambdify([symbols], symb_expr, "numpy")
    # save function to disk this speeds up import drastically
    with open(file_name + ".bin", "wb") as f:
        marshal.dump(func.__code__, f)


def main():
    # define symbols
    w, x, y, z, b_x, b_y, b_z, u_x, u_y, u_z, T_s = sp.symbols("w x y z b_x b_y b_z u_x u_y u_z T_s")
    syms = [T_s, b_x, b_y, b_z, u_x, u_y, u_z, w, x, y, z]

    # state vector
    x_k = np.array([w, x, y, z, b_x, b_y, b_z])
    # orientation quaternion part of state
    q_x_k = Quaternion(*x_k[:4])

    # gyro measurement vector with bias
    u = sp.Matrix([u_x - b_x, u_y - b_y, u_z - b_z])
    u_norm = sp.sqrt(u.dot(u))

    # orientation unit quaternion of gyroscope reading
    u_k = Quaternion.from_axis_angle(u / u_norm, u_norm * T_s)
    sym_to_numpy([T_s, u_x, u_y, u_z, b_x, b_y, b_z], sp.Matrix([u_k.a, u_k.b, u_k.c, u_k.d]), "u")
    # u_fun = sp.lambdify([[T_s, u_x, u_y, u_z, b_x, b_y, b_z]], sp.Matrix([u_k.a, u_k.b, u_k.c, u_k.d]), "numpy")

    # strapdown integration of gyroscope measurements i.e. state value function
    q_f_x = u_k * q_x_k
    f_x = sp.Matrix([q_f_x.a, q_f_x.b, q_f_x.c, q_f_x.d, b_x, b_y, b_z])

    # state transition matrix
    F = f_x.jacobian(x_k)
    sym_to_numpy(syms, F, "F")
    # F_func = sp.lambdify([syms], F, "numpy")

    # reference vectors for acceleration and magentic field
    a_ref = q_x_k.conjugate() * Quaternion(0, 0, 0, 9.81) * q_x_k
    m_ref = q_x_k.conjugate() * Quaternion(0, 0, 1, 0) * q_x_k

    # measurement function
    h_x = sp.Matrix([a_ref.b, a_ref.c, a_ref.d, m_ref.b, m_ref.c, m_ref.d])
    sym_to_numpy([q_x_k.a , q_x_k.b, q_x_k.c, q_x_k.d], h_x, "h")
    # h_fun = sp.lambdify([[q_x_k.a , q_x_k.b, q_x_k.c, q_x_k.d]], h_x, "numpy")

    # first order taylor approx. of measurement function
    H = h_x.jacobian(x_k)
    sym_to_numpy(x_k[:4], H, "H")
    # H_func = sp.lambdify([x_k[:4]], H, "numpy")


if __name__ == '__main__':
    main()

