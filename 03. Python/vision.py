from imports import *

points_list = Queue(maxsize=3)
too_close_flag = Event()

class VisionProcess(Process):
    def __init__(self):
        super().__init__()
        self._x_len = 240
        self._y_len = 180
        self._image = cv.imread(
            "images/tof1.png", cv.IMREAD_GRAYSCALE
        )

    def pava(self, image) -> bool:
        for col in range(self._x_len):
            a = image[:, col]
            a = a[::-1]
            w = np.ones(self._y_len)
            a1 = np.zeros(self._y_len)
            w1 = np.zeros(self._y_len)
            S = np.zeros(self._y_len+1, dtype=int)
            y = np.zeros(self._y_len)

            a1[0] = a[0]
            w1[0] = w[0]

            S[0] = 0
            S[1] = 1

            j = 0
            for i in range(1, self._y_len):
                j += 1

                a1[j] = a[i]
                w1[j] = w[i]

                while j > 0 and a1[j] < a1[j-1]:
                    a1[j-1] = (w1[j] * a1[j] + w1[j-1] * a1[j-1]) / (w1[j-1] + w1[j])
                    w1[j-1] = w1[j] + w1[j-1]
                    j -= 1
                
                S[j+1] = i
            
            S[j+1] = self._y_len
            
            for k in range(1, j+2):
                for l in range(S[k-1], S[k]):
                    y[l] = a1[k-1]
            
            y = y[::-1]
            image[:, col] = y

        self._image = image

        return True
    
    def safe_zone(self, image) -> bool:
        a_prev = image[:, 1]
        a_prev = a_prev[::-1]
        for col in range(self._x_len):
            a = image[:, col]
            a = a[::-1]

            deriv = np.diff(a)

            count = 0
            for i in range(self._y_len-1):
                if deriv[i] < 1:
                    count += 1

                    if count > 15:
                        a[i::] = 0
                        break
                
                else: count = 0

            for i in range(self._y_len-1):
                if a_prev[i] == a_prev[i-1] == a_prev[i+1] == 0:
                    a[i] = 0

            a_prev = a
            a = a[::-1]
            image[:, col] = a

        if not np.count_nonzero(image):
            print("Processing too harsh for this image")
            return False
        
        self._image = image

        return True
    
    def check_distance(self, image) -> bool:
        if image[90, 120] > 240:
            too_close_flag.set()
            # print("Too close!")
            return False
        return True
    
    def extract_points(self, image) -> bool:
        rows = [90, 120, 150]
        points = []
        for i in rows:
            a = image[i, :]
            left_margin = self._x_len
            right_margin = 0
            
            for j in range(self._x_len):
                if a[j] != 0:
                    left_margin = j
                    break
            
            for j in range(self._x_len):
                if a[self._x_len - j - 1] != 0:
                    right_margin = self._x_len - j - 1
                    break
            
            cv.circle(image, (int((left_margin + right_margin)/2), i), 2, color=(0, 0, 255))
            point = [int((left_margin + right_margin)/2), int(a[int((left_margin + right_margin)/2)])]
            points.append(point)
            points_list.put(point)
            
            # print(f"{i}: {left_margin} {right_margin}")
            # print(f"{i}: {point}")
        
        if points[0][0] == points[1][0] == points[2][0]:
            print("Failed identifying points")
            return False
        
        return True
    
    def run(self):
        too_close_flag.clear()
        cd = self.check_distance(self._image)
        # print("CD")
        if not cd: return False

        pava = self.pava(self._image)
        # print("PAVA")
        if not pava: return False

        sz = self.safe_zone(self._image)
        # print("SZ")
        if not sz: return False

        ep = self.extract_points(self._image)
        # print("EP")
        if not ep: return False

        while True:
            cv.imshow("ToF", self._image)
            cv.waitKey(1)

if __name__ == "__main__":
    vp = VisionProcess()
    vp.daemon = True
    vp.start()
    vp.join()