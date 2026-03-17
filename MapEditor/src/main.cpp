#include "ui/MainWindow.h"
#include <QApplication>
#include <QSurfaceFormat>

int main(int argc, char* argv[]) {
    // OpenGL 3.3 Core Profile
    QSurfaceFormat fmt;
    fmt.setVersion(3, 3);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    fmt.setDepthBufferSize(24);
    fmt.setSamples(4);
    QSurfaceFormat::setDefaultFormat(fmt);

    QApplication app(argc, argv);
    app.setApplicationName("MapEditor");
    app.setApplicationVersion("1.0.0");

    // 设置中文字体友好的样式
    app.setStyleSheet(R"(
        QMainWindow, QDialog { font-size: 13px; }
        QTreeWidget { font-size: 12px; }
        QGroupBox { font-weight: bold; margin-top: 8px; }
        QGroupBox::title { subcontrol-position: top left; padding: 4px; }
    )");

    me::MainWindow window;
    window.show();

    // 如果命令行提供了 json 路径, 自动加载
    if (argc > 1) {
        // 使用 QTimer::singleShot 在事件循环启动后加载
        QMetaObject::invokeMethod(&window, [&window, argv]() {
            // 直接调用 onOpen 逻辑
            // 简单起见：显示后用户手动打开
        }, Qt::QueuedConnection);
    }

    return app.exec();
}
