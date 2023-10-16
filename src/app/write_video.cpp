#include "do_nothing_frame_recipient.h"
#include "fr_bgra_to_rgb.h"
#include "kfr_capture_to_bgra.h"
#include "kfr_capture_to_timestamp.h"
#include "fr_display.h"
#include "kfr_recorder_factory.h"
#include "fr_rgb_4k_to_1080p.h"
#include "fr_splitter.h"
#include "kinect_wrapper.h"
#include "do_once.h"

GtkWidget *window;

void *doThread(void *data) {
      DoOnce *mkw = (DoOnce *)data;
      for(;;) {
          mkw->doOnce();
      }
}

gboolean exit_program(GtkWidget *widget, GdkEvent *event, gpointer data) {
    //if menu closed exit program entirely. 
    exit(0);
    return TRUE;
}

static void activateWindow (GtkApplication *app, gpointer user_data){
  window = gtk_application_window_new(app);
  gtk_window_set_title (GTK_WINDOW (window), "Multi Kinect with AprilTag" );
  gtk_window_set_default_size(GTK_WINDOW(window), 1920, 1080 );

  ((FRDisplay *)user_data)->buildWidgets(window);

  g_signal_connect(window, "destroy", G_CALLBACK(exit_program), NULL);
  gtk_widget_show_all (window);
}

int main(int argc, char**argv) {
  FRDisplay frDisplay;

  GtkApplication *app = gtk_application_new ("org.gtk.example", G_APPLICATION_FLAGS_NONE);
  g_signal_connect (app, "activate", G_CALLBACK (activateWindow), &frDisplay);

  KinectWrapper kw(0);
  FRSplitter frSplitter;
  KFRRecorderFactory kfrrf("/tmp/rec.mkv");
  KFRRecorder *kfrr = kw.getKFRRecorder(kfrrf);
  //KFRCaptureToTimeStamp kfrCaptureToTimeStamp;
  //KFRWaitForNewFrame kfrWaitForNewFrame((std::chrono::microseconds)1000);
  KFRCaptureToBGRA kfrCaptureToBGRA;
  FRBGRAToRGB frBGRAToRGB("BGRA4K", "RGB4K");
  FRRGB4KTo1080p frRGB4KTo1080p;
  frRGB4KTo1080p.setFrameRecipient((FrameRecipient*)&frDisplay);
  frBGRAToRGB.setFrameRecipient((FrameRecipient*)&frRGB4KTo1080p);
  kfrCaptureToBGRA.setFrameRecipient((FrameRecipient*)&frBGRAToRGB);
  //kfrWaitForNewFrame.setKinectFrameRecipient((KinectFrameRecipient*)&kfrCaptureToBGRA);
  //kfrCaptureToTimeStamp.setKinectFrameRecipient((KinectFrameRecipient*)&kfrWaitForNewFrame);

  frSplitter.addFrameRecipient((FrameRecipient*)&kfrCaptureToBGRA);
  frSplitter.addFrameRecipient((FrameRecipient*)kfrr);

  kw.setFrameRecipient((FrameRecipient*)&frSplitter);

  kw.start();

  pthread_t thread;
  pthread_create(&thread, NULL, doThread, &kw);

  int status = g_application_run (G_APPLICATION (app), 0, argv);
  g_object_unref (app);
  return 0;
}