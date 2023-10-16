#include "fr_display.h"
#include "data_packet.h"

#include <iostream>
using namespace std;

FRDisplay::FRDisplay() : _initialized(false),
    _darea(NULL),
    _buf(new unsigned char[1920*1080*3]) {
}

FRDisplay::~FRDisplay() {
}

void FRDisplay::buildWidgets(GtkWidget *container) {
    _darea = gtk_drawing_area_new();
    gtk_container_add(GTK_CONTAINER (container), (GtkWidget * ) _darea );
    g_signal_connect (G_OBJECT (_darea), "draw", G_CALLBACK (drawCallback), this);
    _initialized = true;
}

void FRDisplay::initializePacket(DataPacket *dp) {}

void FRDisplay::receiveFrame(DataPacket *dp) {
    //cout << "FRDisplay::receiveFrame" << endl;
    memcpy(_buf, dp->_mats["RGB1080p"].data, 1920*1080*3);
    //Draw each area
    gtk_widget_queue_draw(_darea);
}

gboolean FRDisplay::drawCallback(GtkWidget *widget, cairo_t *cr, gpointer data) {
    FRDisplay *display = (FRDisplay *)data;
    return display->doDraw(cr);
}

gboolean FRDisplay::doDraw(cairo_t *cr) {
    //cout << "FRDisplay::doDraw" << endl;
    if(_initialized) {
        GdkPixbuf *pixbuf = gdk_pixbuf_new_from_data(
            (guint8*)(_buf),
            GDK_COLORSPACE_RGB,
            false,
            8,
            1920,
            1080,
            (int)1920 * 3, NULL, NULL);
        gdk_cairo_set_source_pixbuf(cr, pixbuf, 0, 0);
        cairo_paint(cr);
    }
}