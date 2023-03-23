import numpy as np

from helper import chunks, transform_to_translation, tfs_to_velocity, docs_in_cursor

def min_max_metric(tfs, threashold):
    if docs_in_cursor(tfs) == 0:
        print("No TFs in given Cursor")
        return
    window_size = 10
    x_t = 0
    y_t = 0
    z_t = 0
    for chk in chunks(tfs, window_size):
        x_s = np.array([transform_to_translation(x["transform"])[0] for x in chk])
        y_s = np.array([transform_to_translation(x["transform"])[1] for x in chk])
        z_s = np.array([transform_to_translation(x["transform"])[2] for x in chk])

        if abs(max(x_s) - min(x_s)) > threashold:
            #print("Threashold X", abs(max(x_s) - min(x_s)))
            x_t +=1
        if abs(max(y_s) - min(y_s)) > threashold:
            #print("Threashold Y", abs(max(y_s) - min(y_s)))
            y_t+=1
        if abs(max(z_s) - min(z_s)) > threashold:
            #print("Threashold Z", abs(max(z_s) - min(z_s)))
            z_t+=1
    return x_t, y_t, z_t


def vel_metric(tfs, threashold):
    if docs_in_cursor(tfs) == 0:
        print("No TFs in given Cursor")
        return
    result = 0
    prev = np.array([0, 0, 0])
    for vel in tfs_to_velocity(tfs):
        norm = np.linalg.norm(vel)
        if norm == 0:
            v = np.array([0, 0, 0])
        else:
            v = vel / norm

        diff = v - prev
        if np.linalg.norm(diff) > threashold / norm:
            result += 1
        prev = v
    return result
