from imports import *

class VisionProcess():
    def __init__(self):
        self._x_len = 240
        self._y_len = 180
        # self._image = image

    def pava(self, a, a_len):
        w = np.ones(a_len)

        a1 = np.zeros(a_len)
        w1 = np.zeros(a_len)
        S = np.zeros(a_len+1, dtype=int)
        y = np.zeros(a_len)

        a1[0] = a[0]
        w1[0] = w[0]

        S[0] = 0
        S[1] = 1

        j = 0
        for i in range(1, a_len):
            j += 1

            a1[j] = a[i]
            w1[j] = w[i]

            while j > 0 and a1[j] < a1[j-1]:
                a1[j-1] = (w1[j] * a1[j] + w1[j-1] * a1[j-1]) / (w1[j-1] + w1[j])
                w1[j-1] = w1[j] + w1[j-1]
                j -= 1
            
            S[j+1] = i
        
        S[j+1] = a_len
        
        for k in range(1, j+2):
            for l in range(S[k-1], S[k]):
                y[l] = a1[k-1]

        return y

if __name__ == "__main__":
    vp = VisionProcess()

    print(vp.pava(np.array([1, 2, 3, 2, 1, 4, 5]), 7))

