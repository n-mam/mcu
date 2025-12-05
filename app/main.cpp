#include <QFont>
#include <QIcon>
#include <QQmlContext>
#include <QGuiApplication>
#include <QQmlApplicationEngine>

#include <mcl/log.h>
#include <UdpServerManager.h>
#include <TcpServerManager.h>
#include <UdpServerManager.h>
#include <SerialPortManager.h>
#include <SerialPortManager.h>

int main(int argc, char *argv[]) {

    #if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    #endif

    qputenv("QT_QUICK_CONTROLS_STYLE", QByteArray("Material"));
    qputenv("QT_QUICK_CONTROLS_MATERIAL_THEME", QByteArray("Light"));

    QGuiApplication app(argc, argv);
    QQmlApplicationEngine engine;

    engine.rootContext()->setContextProperty("netManager", getInstance<UdpServerManager>());
    engine.rootContext()->setContextProperty("serialManager", getInstance<SerialPortManager>());

    app.setWindowIcon(QIcon(":app.ico"));
    QFont font("Consolas", 10);
    app.setFont(font);

    const QUrl url(u"qrc:/main.qml"_qs);
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated, &app,
        [url](QObject *obj, const QUrl &objUrl) {
            if (!obj && url == objUrl)
                QCoreApplication::exit(-1);
        }, Qt::QueuedConnection);

    mcl::log::setLogLevel(mcl::log::debug);
    mcl::log::setLogSink<std::string>(
        [](int level, int sink, auto log) {
            getInstance<UdpServerManager>()->logToQml(log);
        });

    engine.load(url);
    return app.exec();
}
