from imports import *

WIDTH_MEAN_SIZE = 2
HEIGHT_MEAN_SIZE = 2


def three_dim_env_thread(image):
        global P, dist

        height_resized, width_resized = (180, 240)
        # print(width, height)

        FOV = 70
        FOV_rad = np.deg2rad(FOV)

        diagonal = np.sqrt(width_resized**2 + height_resized**2)
        focal = (diagonal / 2) / np.tan(FOV_rad / 2)

        focal_width = focal_height = focal
        center_width = width_resized // 2
        center_height = height_resized // 2

        v, u = np.indices((height_resized, width_resized))
        x_n = (u - center_width) / focal_width
        y_n = (v - center_height) / focal_height

        D = image.astype(np.float32)

        rx = x_n
        ry = y_n
        rz = np.ones_like(x_n)

        norm = np.sqrt(rx*rx + ry*ry + rz*rz)
        rx /= norm
        ry /= norm
        rz /= norm

        X = rx * D
        Y = - ry * D
        Y -= np.min(Y)
        Z = rz * D

        P = np.stack([X, Z, Y], axis=-1)
        Y = P[..., 2] - 0.22 > 0
        # mask = Y > 0
        P = P[Y]
        P = P.reshape(-1, 3)
        dist = np.linalg.norm(P, axis=1)

        # print(P)

        # print(contours)

