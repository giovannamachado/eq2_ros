from importlib import import_module


versions = {"cv2":[5,0,0],
            "numpy":[1,26,4],
            "mediapipe":[1,0,1],}

for module,version in versions.items():
    module = import_module(module)
    ver = [int(x) if x.isnumeric else x for x in module.__version__.split(".")]
    right = all(ver[i] == version[i] for i in range(min(len(ver),len(version))))
    print(f"{module}:\n\tCorrect version{right}\n\tExpected:{version}\n\tRecived :{ver}")
#print(cv2.__version__)
#print(numpy.__version__)
#print(mediapipe.__version__)