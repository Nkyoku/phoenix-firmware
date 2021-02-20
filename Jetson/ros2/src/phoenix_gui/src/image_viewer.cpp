#include "image_viewer.hpp"
#include <QtCore/QMetaObject>
#include <QtGui/QMouseEvent>
#include <QtGui/QOpenGLTexture>
#include <QtGui/QOpenGLPixelTransferOptions>
#include <sensor_msgs/image_encodings.hpp>
#include <QtCore/QDebug>
ImageViewerWidget::ImageViewerWidget(QWidget *parent)
    : QOpenGLWidget(parent), QOpenGLFunctions() {
    m_PixelTransferOptions = new QOpenGLPixelTransferOptions();
}

ImageViewerWidget::~ImageViewerWidget() {
    delete m_Texture;
}

void ImageViewerWidget::setMirror(bool horizontal, bool vertical) {
    m_MirrorHorizontal = horizontal;
    m_MirrorVertical = vertical;
}

void ImageViewerWidget::convertBgrToRgb(bool enable) {
    m_BgrFlag = enable;
}

void ImageViewerWidget::useMouse(bool enable) {
    m_MouseNotify = enable;
    setMouseTracking(enable);
}

QSize ImageViewerWidget::optimumImageSize(int original_width, int original_height) {
    double ratio_image = static_cast<double>(original_width) / static_cast<double>(original_height);
    double ratio_widget = static_cast<double>(width()) / static_cast<double>(height());
    double optimum_width, optimum_height;
    if (ratio_image < ratio_widget) {
        // 表示画像の縦幅をウィジェットに合わせる
        optimum_width = ratio_image / ratio_widget * width();
        optimum_height = height();
    } else {
        // 表示画像の横幅をウィジェットに合わせる
        optimum_width = width();
        optimum_height = ratio_widget / ratio_image * height();
    }
    return QSize(static_cast<int>(round(optimum_width)), static_cast<int>(round(optimum_height)));
}

void ImageViewerWidget::initializeGL(void) {
    initializeOpenGLFunctions();
    glClearColor(0.0f, 0.5f, 0.5f, 1.0f);
}

void ImageViewerWidget::resizeGL(int width, int height) {
    glViewport(0, 0, width, height);
    glLoadIdentity();
    glOrtho(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0);
}

void ImageViewerWidget::paintGL(void) {
    // 表示画像が更新されたか確認する
    auto image = _Image;
    if (_NewImage == true) {
        _NewImage = false;

        // ImageをOpenGLのテクスチャに変換する
        if (image) {
            if ((m_Texture == nullptr) || (m_Texture->width() != image->width) || (m_Texture->height() != image->height)) {
                delete m_Texture;
                m_Texture = new QOpenGLTexture(QOpenGLTexture::Target2D);
                m_Texture->setSize(image->width, image->height);
                m_Texture->setFormat(QOpenGLTexture::RGBA8_UNorm);
                m_Texture->allocateStorage(QOpenGLTexture::RGBA, QOpenGLTexture::UInt8);
                m_Texture->setMinMagFilters(QOpenGLTexture::Linear, QOpenGLTexture::Linear);
                m_Texture->setWrapMode(QOpenGLTexture::ClampToEdge);
            }
            QOpenGLTexture::PixelFormat pixel_format;
            int channels = sensor_msgs::image_encodings::numChannels(image->encoding);
            int bit_depth = sensor_msgs::image_encodings::bitDepth(image->encoding);
            switch (channels) {
            case 1:
                pixel_format = QOpenGLTexture::Luminance;
                break;
            case 2:
                pixel_format = QOpenGLTexture::RG;
                break;
            case 3:
                pixel_format = (m_BgrFlag == true) ? QOpenGLTexture::BGR : QOpenGLTexture::RGB;
                break;
            case 4:
                pixel_format = (m_BgrFlag == true) ? QOpenGLTexture::BGRA : QOpenGLTexture::RGBA;
                break;
            default:
                Q_ASSERT(false);
            }
            QOpenGLTexture::PixelType pixel_type;
            switch (bit_depth) {
            case 8:
                pixel_type = QOpenGLTexture::UInt8;
                break;
            case 16:
                pixel_type = QOpenGLTexture::UInt16;
                break;
            case 32:
                pixel_type = QOpenGLTexture::Float32;
                break;
            default:
                Q_ASSERT(false);
            }
            m_PixelTransferOptions->setSwapBytesEnabled(image->is_bigendian);
            if (image->step & 0x1) {
                m_PixelTransferOptions->setAlignment(1);
            } else if (image->step & 0x2) {
                m_PixelTransferOptions->setAlignment(2);
            } else {
                m_PixelTransferOptions->setAlignment(4);
            }
            m_Texture->setData(pixel_format, pixel_type, image->data.data(), m_PixelTransferOptions);
        } else {
            delete m_Texture;
            m_Texture = nullptr;
        }
    }

    // 表示画像があれば描画する
    glClear(GL_COLOR_BUFFER_BIT);
    if (m_Texture != nullptr) {
        // アスペクト比を調整する
        double normalized_width, normalized_height;
        double ratio_image = static_cast<double>(image->width) / static_cast<double>(image->height);
        double ratio_widget = static_cast<double>(width()) / static_cast<double>(height());
        if (ratio_image < ratio_widget) {
            // 表示画像の縦幅をウィジェットに合わせる
            normalized_width = ratio_image / ratio_widget;
            normalized_height = 1.0;
        } else {
            // 表示画像の横幅をウィジェットに合わせる
            normalized_width = 1.0;
            normalized_height = ratio_widget / ratio_image;
        }
        if (m_MirrorHorizontal){
            normalized_width = -normalized_width;
        }
        if (m_MirrorVertical) {
            normalized_height = -normalized_height;
        }

        // 描画する
        glEnable(GL_TEXTURE_2D);
        m_Texture->bind();
        glBegin(GL_QUADS);
        glTexCoord2d(0.0, 1.0);
        glVertex3d(-normalized_width, -normalized_height, 0.0);
        glTexCoord2d(1.0, 1.0);
        glVertex3d(normalized_width, -normalized_height, 0.0);
        glTexCoord2d(1.0, 0.0);
        glVertex3d(normalized_width, normalized_height, 0.0);
        glTexCoord2d(0.0, 0.0);
        glVertex3d(-normalized_width, normalized_height, 0.0);
        glEnd();
        glDisable(GL_TEXTURE_2D);
    }
}

void ImageViewerWidget::mouseMoveEvent(QMouseEvent *event) {
    if (m_MouseNotify == true) {
        QPoint point = toImagePosition(event);
        emit mouseMoved(point.x(), point.y());
    }
}

void ImageViewerWidget::mousePressEvent(QMouseEvent *event) {
    if (m_MouseNotify == true) {
        QPoint point = toImagePosition(event);
        emit mousePressed(point.x(), point.y(), event->button());
    }
}

void ImageViewerWidget::setImage(const std::shared_ptr<sensor_msgs::msg::Image> &image) {
    m_Mutex.lock();
    _Image = image;
    _NewImage = true;
    m_Mutex.unlock();
}

QPoint ImageViewerWidget::toImagePosition(QMouseEvent *event) {
    m_Mutex.lock();
    if (!_Image) {
        m_Mutex.unlock();
        return QPoint(-1, -1);
    }
    double image_width = _Image->width;
    double image_height = _Image->height;
    m_Mutex.unlock();
    QPoint point = event->pos();
    double widget_x = static_cast<double>(point.x()) / static_cast<double>(width());
    double widget_y = static_cast<double>(point.y()) / static_cast<double>(height());
    double normalized_width, normalized_height;
    double ratio_image = image_width / image_height;
    double ratio_widget = static_cast<double>(width()) / static_cast<double>(height());
    if (ratio_image < ratio_widget) {
        // 表示画像の縦幅をウィジェットに合わせる
        normalized_width = ratio_image / ratio_widget;
        normalized_height = 1.0;
    } else {
        // 表示画像の横幅をウィジェットに合わせる
        normalized_width = 1.0;
        normalized_height = ratio_widget / ratio_image;
    }
    int image_x = static_cast<int>(round(((widget_x - 0.5) / normalized_width + 0.5) * image_width));
    int image_y = static_cast<int>(round(((widget_y - 0.5) / normalized_height + 0.5) * image_height));
    return QPoint(image_x, image_y);
}
