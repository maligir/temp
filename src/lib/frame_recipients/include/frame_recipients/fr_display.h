#ifndef FR_DISPLAY_H
#define FR_DISPLAY_H

#include "frame_recipient.h"

#include <gtk/gtk.h>

class FRDisplay : public FrameRecipient {
public:
    FRDisplay();
    ~FRDisplay();
    void initializePacket(DataPacket *kp);
    void receiveFrame(DataPacket *kp);
    void buildWidgets(GtkWidget *container);
    static gboolean drawCallback (GtkWidget *widget, cairo_t *cr, gpointer data);
    gboolean doDraw(cairo_t *cr);
protected:
    bool _initialized;
    GtkWidget *_darea;
    unsigned char *_buf;
};

#endif