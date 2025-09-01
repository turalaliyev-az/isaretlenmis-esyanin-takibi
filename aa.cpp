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

// ===================== FIBONACCI & PRIME (MathUtils) =====================
namespace MathUtils {
    u64 fibonacci(u32 n) {
        if (n == 0) return 0;
        if (n == 1) return 1;
        // Büyüklere karşı overflow riski olduğundan n'i makul tut
        double phi = (1.0 + sqrt(5.0)) / 2.0;
        double psi = (1.0 - sqrt(5.0)) / 2.0;
        double fn = (pow(phi, n) - pow(psi, n)) / sqrt(5.0);
        if (std::isnan(fn) || fn < 0) return 1;
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
        while (count <= n) {
            if (is_prime(num)) count++;
            if (count > n) break;
            num += 2;
            // güvenlik: çok yüksek n'lerde sonsuz döngüyü kır
            if (num > 1000000) break;
        }
        return num;
    }
}

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
const double scale_smoothing = 0.12; // yumuşatma (0..1)

// Optik akış için (kullanılmaya devam edilebilir)
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
            
            // Optik akış noktalarını başlat (isteğe bağlı)
            prevPoints.clear();
            Mat roiColor = safeROI(frame, selection);
            if (!roiColor.empty()) {
                Mat roiGray;
                cvtColor(roiColor, roiGray, COLOR_BGR2GRAY);
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
    
    double widthRatio = currentWidth / (double)referenceObjectSize.width;
    double heightRatio = currentHeight / (double)referenceObjectSize.height;
    
    // Genişlik ve yüksekliğin geometrik ortalaması
    double apparentScale = sqrt(widthRatio * heightRatio);
    return apparentScale;
}

// ===================== ANA PROGRAM =====================
int main() {
    VideoCapture cap;
    cap.open(0, CAP_V4L2);
    if (!cap.isOpened()) cap.open(0, CAP_ANY);
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

    cout << "GERÇEK BOYUT BAZLI NESNE TAKIP - FIB/PRIME ENTEGRE" << endl;

    while (true) {
        bool frameRead = cap.read(frame);
        if (!frameRead) {
            cerr << "Kare alınamadı! Kamera bağlantısını kontrol edin." << endl;
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

        // Görüntü iyileştirme (basit; isteğe göre CLAHE eklenebilir)
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

                // 1) Görünen ölçeği hesapla (apparent scale)
                double apparentScale = calculateScaleFromApparentSize(updatedBox);

                // 2) Fibonacci + Prime tabanlı matematiksel ölçek (mathScale)
                // frameCounter'i sınırlı bir aralığa alıyoruz (ör: 1..30) ki fibonacci çok büyük olmasın
                u32 fibIndex = (frameCounter % 20) + 2;
                u64 fib_val = MathUtils::fibonacci(fibIndex);
                u64 prime_val = MathUtils::nth_prime((frameCounter % 20) + 1);

                // mathScale: 0.5 .. 1.5 aralığına normalize etme (örnek)
                double raw = 1.0;
                if (fib_val > 0 && prime_val > 0) {
                    // (fib * prime) mod 100 üretir. 0..99 -> /100 => 0..0.99
                    double m = double((fib_val * prime_val) % 100) / 100.0;
                    raw = 0.5 + m; // 0.5 .. 1.49
                }
                double mathScale = raw;

                // 3) Tamamen matematiksel uygulama: finalScale = apparentScale * mathScale
                double finalScale = apparentScale * mathScale;

                // 4) Yumuşatma (currentScale ile blend)
                currentScale = currentScale * (1.0 - scale_smoothing) + finalScale * scale_smoothing;

                // 5) Güvenli sınırlar
                currentScale = clamp(currentScale, 0.2, 5.0);

                // 6) Yeni boyutları hesapla ve merkezi koru
                int new_width = static_cast<int>(referenceObjectSize.width * currentScale);
                int new_height = static_cast<int>(referenceObjectSize.height * currentScale);

                new_width = clamp(new_width, 20, frame.cols);
                new_height = clamp(new_height, 20, frame.rows);

                Point center(updatedBox.x + updatedBox.width / 2, updatedBox.y + updatedBox.height / 2);
                selection.x = center.x - new_width / 2;
                selection.y = center.y - new_height / 2;
                selection.width = new_width;
                selection.height = new_height;
                selection &= Rect(0, 0, frame.cols, frame.rows);

                // Görsel geri bildirim
                rectangle(display, selection, Scalar(0, 255, 0), 2);
                putText(display, "TAKIP AKTIF", Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);

                // Bilgiler
                char buf[128];
                snprintf(buf, sizeof(buf), "apparent: %.3f  math: %.3f  final: %.3f", apparentScale, mathScale, currentScale);
                putText(display, buf, Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);

                string size_text = "Boyut: " + to_string(selection.width) + "x" + to_string(selection.height);
                putText(display, size_text, Point(10, 85), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 200, 0), 1);

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

            // RE-DETECT: isteğe göre histogram/optical-flow tabanlı yeniden tespit ekleyebilirsin
            if (frameCounter % reDetectInterval == 0 && isTracking) {
                // (Opsiyonel) implement histogram/feature-match re-detect burada...
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
