
/*
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <iostream>
//
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
*/

























#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <cstdint>
using namespace cv;
using namespace std;

// ===================== TIP TANIMLARI =====================
using u32 = uint32_t;
using u64 = uint64_t;

// ===================== FIBONACCI & PRIME =====================
namespace MathUtils {
    u64 fibonacci(u32 n) {
        if (n == 0) return 0;
        if (n == 1) return 1;
        double phi = (1.0 + sqrt(5.0)) / 2.0;
        double psi = (1.0 - sqrt(5.0)) / 2.0;
        double fn = (pow(phi, n) - pow(psi, n)) / sqrt(5.0);
        return static_cast<u64>(round(fn));
    }

    bool is_prime(u64 n) {
        if (n <= 1) return false;
        if (n <= 3) return true;
        if (n % 2 == 0 || n % 3 == 0) return false;
        for (u64 i = 5; i * i <= n; i += 6) {
            if (n % i == 0 || n % (i + 2) == 0) return false;
        }
        return true;
    }

    u64 nth_prime(u64 n) {
        if (n == 0) return 2;
        u64 count = 1;
        u64 num = 3;
        while (count < n) {
            if (is_prime(num)) {
                count++;
                if (count == n) break;
            }
            num += 2;
        }
        return num;
    }
}

// ===================== GLOBAL DEĞİŞKENLER =====================
Mat frame;
Rect selection;
bool selectObject = false;
bool isTracking = false;
Point origin;
Ptr<Tracker> tracker;
int frameCounter = 0;
const int reDetectInterval = 15;

// Dinamik ROI boyutu için
int initial_width = 0;
int initial_height = 0;
double running_scale = 1.0;
const double scale_smoothing = 0.2; // Ölçek değişimini yumuşatma faktörü

// ===================== MOUSE CALLBACK =====================
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
            initial_width = selection.width;
            initial_height = selection.height;
            running_scale = 1.0; // Ölçek faktörünü sıfırla
            frameCounter = 0; // Sayaç sıfırla
            tracker.release();
            tracker = TrackerCSRT::create();
            tracker->init(frame, selection);
        } else {
            isTracking = false;
            cout << "Geçersiz ROI seçimi. Minimum 20x20 boyutunda olmalı." << endl;
        }
        break;
    }
}

// ===================== HISTOGRAM MATCHING =====================
double histMatch(const Mat& roi, const Mat& search) {
    Mat roi_hsv, search_hsv;
    cvtColor(roi, roi_hsv, COLOR_BGR2HSV);
    cvtColor(search, search_hsv, COLOR_BGR2HSV);

    int h_bins = 30, s_bins = 32;
    int histSize[] = {h_bins, s_bins};
    float h_ranges[] = {0, 180};
    float s_ranges[] = {0, 256};
    const float* ranges[] = {h_ranges, s_ranges};
    int channels[] = {0, 1};

    Mat roi_hist, search_hist;
    calcHist(&roi_hsv, 1, channels, Mat(), roi_hist, 2, histSize, ranges, true, false);
    normalize(roi_hist, roi_hist, 0, 1, NORM_MINMAX);
    calcHist(&search_hsv, 1, channels, Mat(), search_hist, 2, histSize, ranges, true, false);
    normalize(search_hist, search_hist, 0, 1, NORM_MINMAX);

    return compareHist(roi_hist, search_hist, HISTCMP_CORREL);
}

// ===================== ANA PROGRAM =====================
int main() {
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "Kamera açılamadı!" << endl;
        return -1;
    }

    namedWindow("Nesne Takibi", WINDOW_AUTOSIZE);
    setMouseCallback("Nesne Takibi", onMouse, nullptr);

    cout << "Nesne takibi için ekranda bir bölge seçin (sol tıkla sürükle bırak)" << endl;
    cout << "'r': Takibi sıfırla" << endl;
    cout << "'q' veya ESC: Çıkış" << endl;

    while (true) {
        cap >> frame;
        if (frame.empty()) {
            cerr << "Kare alınamadı!" << endl;
            break;
        }

        Mat display;
        frame.copyTo(display);

        // CLAHE ile kontrast artırma
        Mat lab;
        cvtColor(frame, lab, COLOR_BGR2Lab);
        vector<Mat> lab_channels;
        split(lab, lab_channels);
        Ptr<CLAHE> clahe = createCLAHE();
        clahe->setClipLimit(4.0);
        clahe->apply(lab_channels[0], lab_channels[0]);
        merge(lab_channels, lab);
        cvtColor(lab, frame, COLOR_Lab2BGR);

        // Seçim rectangle
        if (selectObject && selection.width > 0 && selection.height > 0) {
            rectangle(display, selection, Scalar(0, 0, 255), 2);
        }

        if (isTracking) {
            frameCounter++;

            if (tracker.empty()) {
                tracker = TrackerCSRT::create();
                tracker->init(frame, selection);
            }

            Rect updatedBox;
            bool ok = tracker->update(frame, updatedBox);
            
            if (ok) {
                // ===================== İYİLEŞTİRİLMİŞ FIB-PRIME TABANLI DYNAMIK SCALE =====================
                // Fibonacci ve asal sayı değerlerini hesapla
                u64 fib_val = MathUtils::fibonacci((frameCounter % 20) + 2);
                u64 prime_val = MathUtils::nth_prime((frameCounter % 20) + 1);
                
                // Ölçek faktörünü hesapla (0.8 - 1.2 aralığında)
                double new_scale = static_cast<double>(fib_val * prime_val % 40) / 100.0 + 0.8;
                
                // Ölçek değişimini yumuşat (smoothing)
                running_scale = running_scale * (1.0 - scale_smoothing) + new_scale * scale_smoothing;
                
                Point center = (updatedBox.tl() + updatedBox.br()) * 0.5;
                int new_width = clamp(static_cast<int>(initial_width * running_scale), 20, frame.cols);
                int new_height = clamp(static_cast<int>(initial_height * running_scale), 20, frame.rows);
                
                selection.x = center.x - new_width / 2;
                selection.y = center.y - new_height / 2;
                selection.width = new_width;
                selection.height = new_height;
                
                // Ekran sınırlarını kontrol et
                selection &= Rect(0, 0, frame.cols, frame.rows);

                rectangle(display, selection, Scalar(0, 255, 0), 2);
                putText(display, "Takip Ediliyor", Point(10, 30), 
                        FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
                
                // Ölçek bilgisini göster
                string scale_text = "Olcek: " + to_string(running_scale).substr(0, 4);
                putText(display, scale_text, Point(10, 60), 
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
                
                // Histogram re-detect
                if (frameCounter % reDetectInterval == 0) {
                    Rect searchWindow = selection;
                    searchWindow.x = max(0, selection.x - selection.width / 2);
                    searchWindow.y = max(0, selection.y - selection.height / 2);
                    searchWindow.width = min(selection.width * 2, frame.cols - searchWindow.x);
                    searchWindow.height = min(selection.height * 2, frame.rows - searchWindow.y);

                    if (searchWindow.width > 0 && searchWindow.height > 0) {
                        Mat searchROI = frame(searchWindow);
                        double matchVal = histMatch(frame(selection), searchROI);

                        if (matchVal > 0.6) {
                            tracker.release();
                            tracker = TrackerCSRT::create();
                            tracker->init(frame, selection);
                        }
                    }
                }
            } else {
                putText(display, "Takip kayboldu!", Point(10, 30),
                        FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
                isTracking = false;
                tracker.release();
            }
        }

        imshow("Nesne Takibi", display);
        int k = waitKey(1);
        if (k == 27 || k == 'q') break;
        else if (k == 'r') {
            isTracking = false;
            tracker.release();
            cout << "Takip sıfırlandı. Yeni bir bölge seçin." << endl;
        }
    }

    cap.release();
    destroyAllWindows();
    return 0;
}