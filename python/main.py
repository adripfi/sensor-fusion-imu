import h5py
import sympy as sp
import numpy as np
import scipy.io as sio
import quaternion

# quaternion from acc mag
def quat_acc_mag(acc, mag):
    z = acc/np.linalg.norm(acc)
    x = -np.cross(z, mag)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)

    return [x.T, y.T, z.T]

def main()
    # Read data
    data = sio.loadmat("data.mat")
    imu = data["imu"][0]
    accA = imu[0][1]["acc"][0, 0]
    gyrA = imu[0][1]["gyr"][0, 0]
    magA = imu[0][1]["mag"][0, 0]

    accC = imu[0][2]["acc"][0, 0]
    gyrC = imu[0][2]["gyr"][0, 0]
    magC = imu[0][2]["mag"][0, 0]

    results = sio.loadmat("results.mat")

    # Remove bias
    rot = quat_acc_mag(accA, magA)

    rate = 100
    N = gyrA.shape[0]

    motion_start = int(18.5 * rate)
    motion_end = int(67.5 * rate)
    motion_duration = motion_end - motion_start

    gyrA_bias_before = np.sum(gyrA[:motion_start, :], axis=0).reshape(1, 3) / motion_start
    gyrA_bias_after = np.sum(gyrA[motion_end:, :], axis=0).reshape(1, 3) / (N - motion_end)

    A_bias_interp = np.array(
        [np.linspace(gyrA_bias_before[:, i], gyrA_bias_after[:, i], motion_duration) for i in range(3)]).T

    bias = np.vstack([np.repeat(gyrA_bias_before, motion_start, axis=0),
                    A_bias_interp[0],
                    np.repeat(gyrA_bias_after, N - motion_end, axis=0)])

    gyrA_nobias = gyrA - bias
    gyrA_nobias_ref = results["gyrA_nobias"]

    return gyrA_nobias, gyrA_nobias_ref


if __name__ == "__main__":
    main()


