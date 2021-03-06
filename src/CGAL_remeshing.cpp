#include <QApplication>
#include <QMimeData>
#include <CGAL/Qt/resources.h>
#include <QLabel>
#include "Main_window.h"

int main(int argc, char **argv) {
  QApplication app(argc, argv);
  app.setOrganizationDomain("cgal.org");
  app.setOrganizationName("CGAL");
  app.setApplicationName("CGALRemeshing");
  //for windows
#if (QT_VERSION >= QT_VERSION_CHECK(5, 3, 0))
  app.setAttribute(Qt::AA_UseDesktopOpenGL);
#endif

  // Import resources from libCGALQt (Qt5).
  CGAL_QT_INIT_RESOURCES;

  MainWindow mainWindow;
  mainWindow.show();
  QStringList args = app.arguments();
  args.removeAt(0);

  if (!args.empty() &&args[0] == "--use-meta") {
    mainWindow.setAddKeyFrameKeyboardModifiers(::Qt::MetaModifier);
    args.removeAt(0);
  }

  Q_FOREACH(QString filename, args)
    mainWindow.open(filename);

  return app.exec();
}

#include "Scene.cpp"
#include "Scene_moc.cpp"
#include "Viewer.cpp"
#include "Viewer_moc.cpp"
#include "Main_window.cpp"
#include "Main_window_moc.cpp"
#include "Isotropic_parameters_moc.cpp"
#include "MinAngle_parameters_moc.cpp"
