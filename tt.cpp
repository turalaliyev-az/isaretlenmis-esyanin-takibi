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
Mat frame, prevFrame, frameGray;
Rect selection;
bool selectObject = false;
bool isTracking = false;
Point origin;
Ptr<Tracker> tracker;
int frameCounter = 0;
const int reDetectInterval = 5; // Daha sık yeniden başlatma

// Nesne ölçümleri için
Size referenceObjectSize; // Piksel cinsinden referans boyut
double currentScale = 1.0;
const double scale_smoothing = 0.2;

// Optik akış için
vector<Point2f> points, prevPoints;
vector<uchar> status;
vector<float> err;

// Takip istatistikleri
int lostFrames = 0;
const int maxLostFrames = 15;

// Son bilinen konum ve ölçek
Rect lastKnownBox;
double lastKnownScale = 1.0;

// Kalman filtresi
KalmanFilter KF;
Mat state, measurement;

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

// ===================== KALMAN FILTRESI BAŞLATMA =====================
void initKalmanFilter() {
    // 4 durum değişkeni (x, y, vx, vy), 2 ölçüm (x, y)
    KF = KalmanFilter(4, 2, 0);
    state = Mat_<float>(4, 1); // [x, y, vx, vy]
    measurement = Mat_<float>(2, 1); // [x, y]

    // Geçiş matrisi (x, y, vx, vy)
    KF.transitionMatrix = (Mat_<float>(4, 4) << 
        1, 0, 1, 0,   // x = x + vx
        0, 1, 0, 1,   // y = y + vy
        0, 0, 1, 0,   // vx sabit
        0, 0, 0, 1);  // vy sabit

    // Ölçüm matrisi (sadece x ve y ölçülüyor)
    KF.measurementMatrix = (Mat_<float>(2, 4) << 
        1, 0, 0, 0,
        0, 1, 0, 0);

    // Hata kovaryansları
    setIdentity(KF.processNoiseCov, Scalar::all(1e-2));
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(KF.errorCovPost, Scalar::all(1));
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
        if (selection.width >= 50 && selection.height >= 50) {
            isTracking = true;
            referenceObjectSize = Size(selection.width, selection.height);
            currentScale = 1.0;
            frameCounter = 0;
            lostFrames = 0;
            lastKnownBox = selection;
            lastKnownScale = 1.0;
            
            // Optik akış noktalarını başlat
            prevPoints.clear();
            Mat roiGray = safeROI(frameGray, selection);
            if (!roiGray.empty()) {
                goodFeaturesToTrack(roiGray, prevPoints, 300, 0.01, 5);
                for (auto& p : prevPoints) {
                    p.x += selection.x;
                    p.y += selection.y;
                }
            }
            
            // Kalman filtresini başlat
            initKalmanFilter();
            state.at<float>(0) = selection.x + selection.width / 2;
            state.at<float>(1) = selection.y + selection.height / 2;
            state.at<float>(2) = 0; // vx
            state.at<float>(3) = 0; // vy
            KF.statePost = state;

            tracker = TrackerCSRT::create();
            tracker->init(frame, selection);
            cout << "Referans boyut ayarlandı: " << referenceObjectSize << " (Piksel)" << endl;
        } else {
            isTracking = false;
            cout << "Geçersiz ROI seçimi. Minimum 50x50 boyutunda olmalı." << endl;
        }
        break;
    }
}

// ===================== GERÇEK BOYUTA GÖRE ÖLÇEK HESAPLAMA =====================
double calculateScaleFromApparentSize(const Rect& currentBox, const Rect2f& flowBox, bool isFastMotion) {
    double widthRatio = 1.0, heightRatio = 1.0;
    
    // CSRT kutusundan ölçek hesapla
    if (currentBox.width > 50 && currentBox.height > 50) {
        widthRatio = static_cast<double>(currentBox.width) / referenceObjectSize.width;
        heightRatio = static_cast<double>(currentBox.height) / referenceObjectSize.height;
    }
    
    // Optik akış kutusundan ölçek hesapla (yalnızca tutarlıysa ve hızlı hareket yoksa)
    double flowWidthRatio = 1.0, flowHeightRatio = 1.0;
    if (!isFastMotion && flowBox.width > 50 && flowBox.height > 50 && 
        abs(flowBox.width - currentBox.width) < 0.5 * referenceObjectSize.width &&
        abs(flowBox.height - currentBox.height) < 0.5 * referenceObjectSize.height) {
        flowWidthRatio = static_cast<double>(flowBox.width) / referenceObjectSize.width;
        flowHeightRatio = static_cast<double>(flowBox.height) / referenceObjectSize.height;
    }
    
    // CSRT ve optik akış oranlarını birleştir (hızlı hareketlerde optik akışın ağırlığı azalır)
    double flowWeight = isFastMotion ? 0.1 : 0.2;
    double combinedWidthRatio = (1.0 - flowWeight) * widthRatio + flowWeight * flowWidthRatio;
    double combinedHeightRatio = (1.0 - flowWeight) * heightRatio + flowWeight * flowHeightRatio;
    
    // Geometrik ortalama
    double apparentScale = sqrt(combinedWidthRatio * combinedHeightRatio);
    
    // Yakınlaşmada küçülmeyi, uzaklaşmada aşırı büyümeyi sınırla
    return clamp(apparentScale, 0.8, 2.0);
}

// ===================== HIZLI HAREKET TESPITI =====================
bool detectFastMotion(const vector<Point2f>& prevPts, const vector<Point2f>& newPts, const vector<uchar>& status) {
    if (prevPts.size() != newPts.size()) return false;
    
    double totalDisplacement = 0.0;
    int validCount = 0;
    for (size_t i = 0; i < status.size(); i++) {
        if (status[i]) {
            double dx = newPts[i].x - prevPts[i].x;
            double dy = newPts[i].y - prevPts[i].y;
            totalDisplacement += sqrt(dx * dx + dy * dy);
            validCount++;
        }
    }
    
    if (validCount == 0) return false;
    double avgDisplacement = totalDisplacement / validCount;
    return avgDisplacement > 20.0; // Piksel cinsinden eşik
}

// ===================== ANA PROGRAM =====================
int main() {
    // Kamera açma ve yeniden bağlanma denemeleri
    VideoCapture cap;
    int retryCount = 0;
    const int maxRetries = 3;
    
    cap.open(0, CAP_V4L2);
    if (!cap.isOpened()) {
        cap.open(0, CAP_ANY);
    }
    
    while (!cap.isOpened() && retryCount < maxRetries) {
        cerr << "Kamera açılamadı, tekrar deneniyor... (" << retryCount + 1 << "/" << maxRetries << ")" << endl;
        cap.open(0, CAP_ANY);
        retryCount++;
        waitKey(1000); // 1 saniye bekle
    }
    
    if (!cap.isOpened()) {
        cerr << "Kamera açılamadı! Kamera bağlantısını kontrol edin." << endl;
        return -1;
    }

    // Kamera ayarları
    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(CAP_PROP_FPS, 30);

    namedWindow("Nesne Takibi", WINDOW_AUTOSIZE);
    setMouseCallback("Nesne Takibi", onMouse, nullptr);

    cout << "==================================================" << endl;
    cout << "GERÇEK BOYUT BAZLI NESNE TAKİP SİSTEMİ" << endl;
    cout << "==================================================" << endl;
    cout << "1. Nesneyi seçmek için sol tıklayıp sürükleyin" << endl;
    cout << "2. Seçilen kare, nesnenin gerçek boyutlarını temsil eder" << endl;
    cout << "3. Nesneyi yakınlaştırdığınızda kutu büyüyecek" << endl;
    cout << "4. Nesneyi uzaklaştırdığınızda kutu küçülecek" << endl;
    cout << "5. 'r': Takibi sıfırla" << endl;
    cout << "6. 'q' veya ESC: Çıkış" << endl;
    cout << "==================================================" << endl;

    while (true) {
        bool frameRead = cap.read(frame);
        if (!frameRead || frame.empty()) {
            cerr << "Kare alınamadı veya boş! Kamera bağlantısını kontrol edin." << endl;
            
            // Kamera bağlantısını yeniden dene
            cap.release();
            retryCount = 0;
            while (!cap.isOpened() && retryCount < maxRetries) {
                cap.open(0, CAP_ANY);
                retryCount++;
                waitKey(1000);
            }
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

        // Görüntü iyileştirme
        cvtColor(frame, frameGray, COLOR_BGR2GRAY);
        equalizeHist(frameGray, frameGray);

        // Seçim rectangle ve + işareti
        if (selectObject && selection.width > 0 && selection.height > 0) {
            rectangle(display, selection, Scalar(0, 0, 255), 2);
            Point center(selection.x + selection.width / 2, selection.y + selection.height / 2);
            line(display, Point(center.x - 10, center.y), Point(center.x + 10, center.y), Scalar(0, 0, 255), 2);
            line(display, Point(center.x, center.y - 10), Point(center.x, center.y + 10), Scalar(0, 0, 255), 2);
        }

        if (isTracking) {
            frameCounter++;

            // Kalman filtresi tahmini
            Mat prediction = KF.predict();
            Point predictedCenter(prediction.at<float>(0), prediction.at<float>(1));

            if (tracker.empty()) {
                tracker = TrackerCSRT::create();
                tracker->init(frame, lastKnownBox);
            }

            Rect updatedBox;
            bool ok = tracker->update(frame, updatedBox);
            Rect2f flowBox(0, 0, 0, 0);
            bool isFastMotion = false;

            // Optik akış ile doğrulama
            if (!prevPoints.empty() && !prevFrame.empty()) {
                vector<Point2f> newPoints;
                calcOpticalFlowPyrLK(prevFrame, frameGray, prevPoints, newPoints, status, err);
                
                int validPoints = count(status.begin(), status.end(), 1);
                isFastMotion = detectFastMotion(prevPoints, newPoints, status);
                
                if (validPoints < prevPoints.size() * 0.5) {
                    lostFrames++;
                } else {
                    // Optik akış ile kutuyu düzelt
                    vector<Point2f> validNewPoints;
                    for (size_t i = 0; i < status.size(); i++) {
                        if (status[i]) validNewPoints.push_back(newPoints[i]);
                    }
                    if (!validNewPoints.empty()) {
                        flowBox = boundingRect(validNewPoints);
                        updatedBox.x = isFastMotion ? 0.9 * updatedBox.x + 0.1 * flowBox.x : 0.8 * updatedBox.x + 0.2 * flowBox.x;
                        updatedBox.y = isFastMotion ? 0.9 * updatedBox.y + 0.1 * flowBox.y : 0.8 * updatedBox.y + 0.2 * flowBox.y;
                    }
                    prevPoints = newPoints;
                }
            }

            if (ok && lostFrames < maxLostFrames) {
                lostFrames = 0;
                
                // Görünen boyuta göre ölçek hesapla
                double newScale = calculateScaleFromApparentSize(updatedBox, flowBox, isFastMotion);
                
                // Ölçek değişimini yumuşat
                currentScale = currentScale * (1.0 - scale_smoothing) + newScale * scale_smoothing;
                
                // Yeni boyutları hesapla
                int new_width = static_cast<int>(referenceObjectSize.width * currentScale);
                int new_height = static_cast<int>(referenceObjectSize.height * currentScale);
                
                // Boyutları sınırla
                new_width = clamp(new_width, 50, frame.cols);
                new_height = clamp(new_height, 50, frame.rows);
                
                // Merkezi Kalman tahmini ile düzelt
                Point trackerCenter(updatedBox.x + updatedBox.width / 2, updatedBox.y + updatedBox.height / 2);
                Point center = isFastMotion ? Point(0.7 * trackerCenter.x + 0.3 * predictedCenter.x, 
                                                  0.7 * trackerCenter.y + 0.3 * predictedCenter.y) :
                                             Point(0.9 * trackerCenter.x + 0.1 * predictedCenter.x, 
                                                  0.9 * trackerCenter.y + 0.1 * predictedCenter.y);
                
                selection.x = center.x - new_width / 2;
                selection.y = center.y - new_height / 2;
                selection.width = new_width;
                selection.height = new_height;
                
                // Ekran sınırlarını kontrol et
                selection &= Rect(0, 0, frame.cols, frame.rows);

                // Son bilinen konumu ve ölçeği güncelle
                lastKnownBox = selection;
                lastKnownScale = currentScale;

                // Kalman filtresini güncelle
                measurement.at<float>(0) = center.x;
                measurement.at<float>(1) = center.y;
                KF.correct(measurement);

                // Takip kutusunu ve + işaretini çiz
                rectangle(display, selection, Scalar(0, 255, 0), 2);
                Point boxCenter(selection.x + selection.width / 2, selection.y + selection.height / 2);
                line(display, Point(boxCenter.x - 10, boxCenter.y), Point(boxCenter.x + 10, boxCenter.y), Scalar(255, 0, 0), 2);
                line(display, Point(boxCenter.x, boxCenter.y - 10), Point(boxCenter.x, boxCenter.y + 10), Scalar(255, 0, 0), 2);
                
                // Bilgileri göster
                putText(display, "TAKIP AKTIF", Point(10, 30), 
                        FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
                
                string scale_text = "Olcek: " + to_string(currentScale).substr(0, 5);
                putText(display, scale_text, Point(10, 60), 
                        FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 0), 2);
                
                string size_text = "Boyut: " + to_string(selection.width) + "x" + to_string(selection.height) + " piksel";
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
                // Nesne görüş alanından çıktığında, son bilinen kutuyu kullan
                selection = lastKnownBox;
                currentScale = lastKnownScale;
                rectangle(display, selection, Scalar(255, 165, 0), 2); // Turuncu kutu
                Point boxCenter(selection.x + selection.width / 2, selection.y + selection.height / 2);
                line(display, Point(boxCenter.x - 10, boxCenter.y), Point(boxCenter.x + 10, boxCenter.y), Scalar(255, 165, 0), 2);
                line(display, Point(boxCenter.x, boxCenter.y - 10), Point(boxCenter.x, boxCenter.y + 10), Scalar(255, 165, 0), 2);
                
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

            // Her reDetectInterval karede tracker’ı yeniden başlat
            if (frameCounter % reDetectInterval == 0 && isTracking) {
                tracker = TrackerCSRT::create();
                tracker->init(frame, lastKnownBox);
                // Optik akış noktalarını yeniden başlat
                prevPoints.clear();
                Mat roiGray = safeROI(frameGray, lastKnownBox);
                if (!roiGray.empty()) {
                    goodFeaturesToTrack(roiGray, prevPoints, 300, 0.01, 5);
                    for (auto& p : prevPoints) {
                        p.x += lastKnownBox.x;
                        p.y += lastKnownBox.y;
                    }
                }
            }
        }

        // Kullanım talimatları
        if (!isTracking) {
            putText(display, "Nesne secmek icin sol tiklayip surukleyin", 
                    Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2);
        }

        imshow("Nesne Takibi", display);
        frameGray.copyTo(prevFrame); // Önceki kareyi güncelle
        
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