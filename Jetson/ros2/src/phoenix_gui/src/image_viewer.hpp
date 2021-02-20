#pragma once

#include <QtCore/QMutex>
#include <QtWidgets/QOpenGLWidget>
#include <QtGui/QOpenGLFunctions>
#include <sensor_msgs/msg/image.hpp>

QT_FORWARD_DECLARE_CLASS(QOpenGLTexture)
QT_FORWARD_DECLARE_CLASS(QOpenGLPixelTransferOptions)

class ImageViewerWidget : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT

public:
    // コンストラクタ
    explicit ImageViewerWidget(QWidget *parent = nullptr);

    // デストラクタ
    ~ImageViewerWidget();

    // 反転する
    void setMirror(bool horizontal, bool vertical);

    // BGRとRGBの変換を行うように設定する
    void convertBgrToRgb(bool enable = true);

    // マウスイベントを使用するように設定する
    void useMouse(bool enable = true);

    // アスペクト比を保って表示できる最適な画像サイズを計算する
    QSize optimumImageSize(int original_width, int original_height);

    // 表示画像を設定する
    // 画像は次の再描画時に更新される
    void setImage(const std::shared_ptr<sensor_msgs::msg::Image> &image);

    // マウスが移動したことを通知するシグナル
    Q_SIGNAL void mouseMoved(int x, int y);

    // マウスボタンが押されたことを通知するシグナル
    Q_SIGNAL void mousePressed(int x, int y, Qt::MouseButton button);

protected:
    // OpenGLの初期化時に呼ばれる
    void initializeGL(void) override;

    // リサイズ時に呼ばれる
    void resizeGL(int width, int height) override;

    // 描画時に呼ばれる
    void paintGL(void) override;

    // マウスが動いたときに呼ばれる
    void mouseMoveEvent(QMouseEvent *event) override;

    // マウスボタンが押されたときに呼ばれる
    void mousePressEvent(QMouseEvent *event) override;

private:
    // m_Imageへのアクセスを調停するミューテックス
    QMutex m_Mutex;

    // テクスチャの転送オプション
    QOpenGLPixelTransferOptions *m_PixelTransferOptions = nullptr;

    // 表示画像を保持するテクスチャ
    QOpenGLTexture *m_Texture = nullptr;

    // 表示画像を保持するImage
    std::shared_ptr<sensor_msgs::msg::Image> _Image;

    // 表示画像が更新されたことを示すフラグ
    bool _NewImage = false;

    /// 水平反転フラグ
    bool m_MirrorHorizontal = false;

    /// 垂直反転フラグ
    bool m_MirrorVertical = false;

    // BGRからRGBへの変換を行うフラグ
    bool m_BgrFlag = false;

    // マウスイベントを通知するフラグ
    bool m_MouseNotify = false;

    // マウス座標から画像上の座標へ変換する
    QPoint toImagePosition(QMouseEvent *event);
};
