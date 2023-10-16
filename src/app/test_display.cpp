#include "do_nothing_frame_recipient.h"
#include "fr_bgra_to_rgb.h"
#include "kfr_capture_to_bgra.h"
#include "fr_display.h"
#include "kfr_recorder_factory.h"
#include "fr_rgb_4k_to_1080p.h"
#include "kinect_wrapper.h"

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

  ((KFRDisplay *)user_data)->buildWidgets(window);

  g_signal_connect(window, "destroy", G_CALLBACK(exit_program), NULL);
  gtk_widget_show_all (window);
}

int main(int argc, char**argv) {
  FRDisplay frDisplay;

  GtkApplication *app = gtk_application_new ("org.gtk.example", G_APPLICATION_FLAGS_NONE);
  g_signal_connect (app, "activate", G_CALLBACK (activateWindow), &frDisplay);

  KinectWrapper kw(0);
  //KFRRecorderFactory kfrrf("/tmp/rec.mkv");
  //KFRRecorder *kfrr = kw.getKFRRecorder(kfrrf);
  KFRCaptureToBGRA kfrCaptureToBGRA;
  FRBGRAToRGB frBGRAToRGB;
  FRRGB4KTo1080p frRGB4KTo1080p;
  frRGB4KTo1080p.setFrameRecipient((FrameRecipient*)&frDisplay);
  frBGRAToRGB.setFrameRecipient((FrameRecipient*)&frRGB4KTo1080p);
  kfrCaptureToBGRA.setFrameRecipient((FrameRecipient*)&frBGRAToRGB);
  kw.setFrameRecipient((FrameRecipient*)&kfrCaptureToBGRA);

  pthread_t thread;
  pthread_create(&thread, NULL, doThread, &kw);

  int status = g_application_run (G_APPLICATION (app), 0, argv);
  g_object_unref (app);
  return 0;
}