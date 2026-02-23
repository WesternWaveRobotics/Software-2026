import cv2
from ultralytics import YOLO

# Initialize custom model
model = YOLO("./models/crabs_exp9.pt")  # yolo26n.pt

# Video stream
cap = cv2.VideoCapture(0)

# Make sure camera is open
if not cap.isOpened():
    print("Cannot open camera")
    exit()

while cap.isOpened():
    # ret (if frame is read correctly returns true)
    ret, frame = cap.read()

    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    # Get results from model, conf(confidence) > 0.25??
    results = model(frame, conf=0.25, verbose=True)

    # For each result create bounding-boxes
    for result in results:
        boxes = result.boxes
        if boxes is None:
            continue

        # Convert boxes to a numpy array (in [x1, y1, x2, y2] formatt)
        xyxy = boxes.xyxy.cpu().numpy()
        names = boxes.cls.cpu().numpy().astype(int)

        for (x1, y1, x2, y2), name in zip(xyxy, names):
            # Draw bounding boxes
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            # Label the name of detected obj
            cv2.putText(
                frame,
                result.names[name],
                (int(x1), int(y1) - 10),
                cv2.FONT_HERSHEY_PLAIN,
                1.25,
                (0, 255, 0),
                2,
            )

    # Display the resulting frame
    cv2.imshow("Live Cameera Feed", frame)

    if cv2.waitKey(1) == ord("q"):
        break


cap.release()
cv2.destroyAllWindows()
