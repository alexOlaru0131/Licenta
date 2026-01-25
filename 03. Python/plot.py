from imports import *

tof_image_queue = Queue()
mean_map_queue = Queue()
track_map_queue = Queue()
track_map_interpolated_queue = Queue()

def plotter():
    plt.ion()

    fig, ax = plt.subplots(2, 2)
    img00 = ax[0, 0].imshow(
        np.zeros((180, 240))
    )
    img01 = ax[0, 1].imshow(
        np.zeros((36, 48))
    )
    img10 = ax[1, 0].imshow(
        np.zeros((36, 48))
    )
    img11 = ax[1, 1].imshow(
        np.zeros((36, 48))
    )


    while True:
        if not tof_image_queue.empty():
            tof_image = tof_image_queue.get()
            img00.set_data(tof_image)
            img00.set_clim(vmin=tof_image.min(), vmax=tof_image.max())

        if not mean_map_queue.empty():
            mean_map = mean_map_queue.get()
            img01.set_data(mean_map)
            img01.set_clim(vmin=mean_map.min(), vmax=mean_map.max())

        if not track_map_queue.empty():
            track_map = track_map_queue.get()
            img10.set_data(track_map)
            img10.set_clim(vmin=track_map.min(), vmax=track_map.max())
        
        if not track_map_interpolated_queue.empty():
            track_map_interpolated = track_map_interpolated_queue.get()
            img11.set_data(track_map_interpolated)
            img11.set_clim(vmin=track_map_interpolated.min(), vmax=track_map_interpolated.max())

        fig.canvas.draw_idle()

        plt.pause(0.01)
