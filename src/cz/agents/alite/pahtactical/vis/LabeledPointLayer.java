package cz.agents.alite.pahtactical.vis;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;

import javax.vecmath.Point2d;

import cz.agents.alite.vis.Vis;
import cz.agents.alite.vis.layer.AbstractLayer;
import cz.agents.alite.vis.layer.VisLayer;

public class LabeledPointLayer extends AbstractLayer {
    public static VisLayer create(final Point2d point, final String label) {
        return new AbstractLayer() {

			@Override
			public void paint(Graphics2D canvas) {
				super.paint(canvas);
				canvas.setStroke(new BasicStroke(1));
				canvas.setColor(Color.red);
				canvas.drawOval(Vis.transX(point.x)-2, Vis.transY(point.y)-2, 4, 4);
				canvas.drawString(label, Vis.transX(point.x)+5, Vis.transY(point.y)+5);
			}
		};
    }
}
