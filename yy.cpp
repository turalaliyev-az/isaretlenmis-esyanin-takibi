#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <iostream>

using namespace cv;
using namespace std;

// Global değişkenler
Mat frame;
Rect selection;
bool selectObject = false;
bool isTracking = false;
Point origin;
Ptr<Tracker> tracker;

int frameCounter = 0;
const int reDetectInterval = 15; // her 15 frame'de re-detect

// Mouse callback
void onMouse(int event, int x, int y, int, void*) {
    if (selectObject) {
        selection.x = min(x, origin.x);
        selection.y = min(y, origin.y);
        selection.width = abs(x - origin.x);
        selection.height = abs(y - origin.y);
        selection &= Rect(0, 0, frame.cols, frame.rows);
    }

    switch (event) {
    case EVENT_LBUTTONDOWN:
        origin = Point(x, y);
        selection = Rect(x, y, 0, 0);
        selectObject = true;
        break;
    case EVENT_LBUTTONUP:
        selectObject = false;
        if (selection.width >= 20 && selection.height >= 20) {
            isTracking = true;
        } else {
            isTracking = false;
            cout << "Geçersiz ROI seçimi." << endl;
        }
        break;
    }
}

int main() {
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "Kamera açılamadı!" << endl;
        return -1;
    }

    namedWindow("Nesne Takibi", WINDOW_NORMAL);
    setMouseCallback("Nesne Takibi", onMouse, nullptr);

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        Mat display = frame.clone();

        // Seçim rectangle'ı çiz
        if (selectObject && selection.width > 0 && selection.height > 0) {
            rectangle(display, selection, Scalar(0, 0, 255), 2);
        }

      
        if (isTracking) {
            frameCounter++;

            // Tracker yoksa oluştur
            if (!tracker) {
                tracker = TrackerCSRT::create();
                tracker->init(frame, selection);
            }

            // Tracker güncelle
            Rect updatedBox;
            bool ok = tracker->update(frame, updatedBox);

            if (ok) {
                selection = updatedBox;
                rectangle(display, selection, Scalar(0, 255, 0), 2);
            } else {
                putText(display, "Takip kayboldu!", Point(10,30),
                        FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,0,255), 2);
                isTracking = false;
                tracker.release();
            }

            // Her reDetectInterval frame'de yeniden tespit
            if (frameCounter % reDetectInterval == 0 && isTracking) {
               
                Rect searchWindow = selection;
                searchWindow.x = max(0, selection.x - selection.width);
                searchWindow.y = max(0, selection.y - selection.height);
                searchWindow.width = min(frame.cols - searchWindow.x, selection.width * 3);
                searchWindow.height = min(frame.rows - searchWindow.y, selection.height * 3);

                Mat searchROI = frame(searchWindow);
                Mat templateROI = frame(selection);

                Mat result;
                matchTemplate(searchROI, templateROI, result, TM_CCOEFF_NORMED);
                double maxVal;
                Point maxLoc;
                minMaxLoc(result, nullptr, &maxVal, nullptr, &maxLoc);

                if (maxVal > 0.7) { 
                    Rect newBox(maxLoc.x + searchWindow.x,
                                maxLoc.y + searchWindow.y,
                                selection.width,
                                selection.height);
                    selection = newBox;

                    tracker.release();
                    tracker = TrackerCSRT::create();
                    tracker->init(frame, selection);
                }
            }
        }

        imshow("Nesne Takibi", display);

        int k = waitKey(1) & 0xFF;
        if (k == 27 || k == 'q') break;
        else if (k == 'r') {
            isTracking = false;
            tracker.release();
        }
    }

    cap.release();
    destroyAllWindows();
    return 0;
}

