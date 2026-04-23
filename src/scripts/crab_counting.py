import cv2
from ultralytics import solutions


def count_crabs(
    cap: cv2.VideoCapture,
    model_path="yolo26n.pt",
    region_pts=[(0, 1000), (1900, 1000), (1900, 0), (0, 0)],
):
    """Count the number of Invasive European Green Crabs in a video stream"""

    # Check video stream is open
    assert cap.isOpened(), "Error: cant open video"

    # Initalizie counter for detections inside defined region
    # docs: https://docs.ultralytics.com/solutions/#solutions
    counter = solutions.ObjectCounter(
        model=model_path, conf=0.25, region=region_pts, device="cpu", show_out=False
    )

    while cap.isOpened():
        ret, frame = cap.read()

        if not ret:
            print("Error: failed to read capture")
            break
        # Count number of EU green crabs inside region
        counter(frame)

        cv2.imshow("Live Camera Feed", frame)

        if cv2.waitKey(1) == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    # For testing purposes
    cap = cv2.VideoCapture(1)
    print(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    count_crabs(cap, model_path="./models/yolo26n.pt")
