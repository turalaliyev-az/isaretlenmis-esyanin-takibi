
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
#include <opencv2/video/tracking.hpp>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <cstdint>
using namespace cv;
using namespace std;

// ===================== TIP TANIMLARI =====================
using u32 = uint32_t;
using u64 = uint64_t;

// ===================== GLOBAL DEĞİŞKENLER =====================
Mat frame, prevFrame;
Rect selection;
bool selectObject = false;
bool isTracking = false;
Point origin;
Ptr<Tracker> tracker;
int frameCounter = 0;
const int reDetectInterval = 10;

// Nesne ölçümleri için
Size referenceObjectSize;
double currentScale = 1.0;
const double scale_smoothing = 0.1;

// Optik akış için
vector<Point2f> points, prevPoints;
vector<uchar> status;
vector<float> err;

// Takip istatistikleri
int lostFrames = 0;
const int maxLostFrames = 15;

// ===================== GÜVENLİ ROI ALMA =====================
Mat safeROI(const Mat& img, const Rect& roi) {
    Rect safeRoi = roi;
    safeRoi.x = max(0, safeRoi.x);
    safeRoi.y = max(0, safeRoi.y);
    safeRoi.width = min(safeRoi.width, img.cols - safeRoi.x);
    safeRoi.height = min(safeRoi.height, img.rows - safeRoi.y);
    
    if (safeRoi.width <= 5 || safeRoi.height <= 5) {
        return Mat();
    }
    
    return img(safeRoi);
}

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
            referenceObjectSize = Size(selection.width, selection.height);
            currentScale = 1.0;
            frameCounter = 0;
            lostFrames = 0;
            
            // Optik akış noktalarını başlat
            prevPoints.clear();
            Mat roiGray = safeROI(frame, selection);
            if (!roiGray.empty()) {
                cvtColor(roiGray, roiGray, COLOR_BGR2GRAY);
                goodFeaturesToTrack(roiGray, prevPoints, 300, 0.01, 5);
                for (auto& p : prevPoints) {
                    p.x += selection.x;
                    p.y += selection.y;
                }
            }
            
            tracker = TrackerCSRT::create();
            tracker->init(frame, selection);
            cout << "Referans boyut ayarlandı: " << referenceObjectSize << endl;
        } else {
            isTracking = false;
            cout << "Geçersiz ROI seçimi. Minimum 20x20 boyutunda olmalı." << endl;
        }
        break;
    }
}

// ===================== GERÇEK BOYUTA GÖRE ÖLÇEK HESAPLAMA =====================
double calculateScaleFromApparentSize(const Rect& currentBox) {
    double currentWidth = currentBox.width;
    double currentHeight = currentBox.height;
    
    double widthRatio = currentWidth / referenceObjectSize.width;
    double heightRatio = currentHeight / referenceObjectSize.height;
    
    // Genişlik ve yüksekliğin geometrik ortalamasını al
    double apparentScale = sqrt(widthRatio * heightRatio);
    
    return apparentScale;
}

// ===================== ANA PROGRAM =====================
int main() {
    // Farklı kamera API'leri dene
    VideoCapture cap;
    
    // Önce V4L2 ile dene (Linux için)
    cap.open(0, CAP_V4L2);
    if (!cap.isOpened()) {
        // V4L2 çalışmazsa ANY ile dene
        cap.open(0, CAP_ANY);
    }
    
    if (!cap.isOpened()) {
        cerr << "Kamera açılamadı! Kamera bağlantısını kontrol edin." << endl;
        return -1;
    }

    // Basit kamera ayarları
    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(CAP_PROP_FPS, 30);

    namedWindow("Nesne Takibi", WINDOW_AUTOSIZE);
    setMouseCallback("Nesne Takibi", onMouse, nullptr);

    cout << "==================================================" << endl;
    cout << "GERÇEK BOYUT BAZLI NESNE TAKİP SİSTEMİ" << endl;
    cout << "==================================================" << endl;
    cout << "1. Nesneyi seçmek için sol tıklayıp sürükleyin" << endl;
    cout << "2. Seçilen nesnenin boyutu referans alınacaktır" << endl;
    cout << "3. Nesneyi yakınlaştırdığınızda kutu büyüyecek" << endl;
    cout << "4. Nesneyi uzaklaştırdığınızda kutu küçülecek" << endl;
    cout << "5. 'r': Takibi sıfırla" << endl;
    cout << "6. 'q' veya ESC: Çıkış" << endl;
    cout << "==================================================" << endl;

    while (true) {
        bool frameRead = cap.read(frame);
        if (!frameRead) {
            cerr << "Kare alınamadı! Kamera bağlantısını kontrol edin." << endl;
            
            // Kamera bağlantısını yeniden dene
            cap.release();
            cap.open(0, CAP_ANY);
            if (!cap.isOpened()) {
                cerr << "Kamera yeniden bağlanamadı. Program sonlandırılıyor." << endl;
                break;
            }
            cap.set(CAP_PROP_FRAME_WIDTH, 640);
            cap.set(CAP_PROP_FRAME_HEIGHT, 480);
            continue;
        }

        Mat display;
        frame.copyTo(display);

        // Görüntü iyileştirme (daha basit)
        Mat frameGray;
        cvtColor(frame, frameGray, COLOR_BGR2GRAY);
        equalizeHist(frameGray, frameGray);
        cvtColor(frameGray, frame, COLOR_GRAY2BGR);

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
                lostFrames = 0;
                
                // Görünen boyuta göre ölçek hesapla
                double newScale = calculateScaleFromApparentSize(updatedBox);
                
                // Ölçek değişimini yumuşat
                currentScale = currentScale * (1.0 - scale_smoothing) + newScale * scale_smoothing;
                
                // Ölçeği makul sınırlarda tut
                currentScale = clamp(currentScale, 0.2, 5.0);
                
                // Yeni boyutları hesapla
                int new_width = static_cast<int>(referenceObjectSize.width * currentScale);
                int new_height = static_cast<int>(referenceObjectSize.height * currentScale);
                
                // Boyutları sınırla
                new_width = clamp(new_width, 20, frame.cols);
                new_height = clamp(new_height, 20, frame.rows);
                
                // Merkezi koruyarak yeni konumu hesapla
                Point center(updatedBox.x + updatedBox.width/2, updatedBox.y + updatedBox.height/2);
                selection.x = center.x - new_width / 2;
                selection.y = center.y - new_height / 2;
                selection.width = new_width;
                selection.height = new_height;
                
                // Ekran sınırlarını kontrol et
                selection &= Rect(0, 0, frame.cols, frame.rows);

                // Takip kutusunu çiz
                rectangle(display, selection, Scalar(0, 255, 0), 2);
                
                // Bilgileri göster
                putText(display, "TAKIP AKTIF", Point(10, 30), 
                        FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
                
                string scale_text = "Olcek: " + to_string(currentScale).substr(0, 5);
                putText(display, scale_text, Point(10, 60), 
                        FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 0), 2);
                
                string size_text = "Boyut: " + to_string(selection.width) + "x" + to_string(selection.height);
                putText(display, size_text, Point(10, 90), 
                        FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 200, 0), 2);
                
                // Yakınlaşma/uzaklaşma durumu
                if (currentScale < 0.95) {
                    putText(display, "UZAKLASIYOR", Point(10, 120), 
                            FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 100, 255), 2);
                } else if (currentScale > 1.05) {
                    putText(display, "YAKINLASIYOR", Point(10, 120), 
                            FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 100), 2);
                }
                
            } else {
                lostFrames++;
                putText(display, "TAKIP ZAYIF " + to_string(lostFrames) + "/" + to_string(maxLostFrames), 
                        Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
                
                if (lostFrames >= maxLostFrames) {
                    isTracking = false;
                    tracker.release();
                    prevPoints.clear();
                    putText(display, "TAKIP DURDURULDU", Point(10, 60), 
                            FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
                }
            }
        }

        // Kullanım talimatları
        if (!isTracking) {
            putText(display, "Nesne secmek icin sol tiklayip surukleyin", 
                    Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2);
        }

        imshow("Nesne Takibi", display);
        int k = waitKey(30);
        if (k == 27 || k == 'q') break;
        else if (k == 'r') {
            isTracking = false;
            tracker.release();
            prevPoints.clear();
            cout << "Takip sıfırlandı. Yeni nesne seçin." << endl;
        }
    }

    cap.release();
    destroyAllWindows();
    return 0;
}