package cz.agents.alite.pahtactical.vis;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;

import javax.vecmath.Point2d;

import cz.agents.alite.vis.Vis;
import cz.agents.alite.vis.layer.AbstractLayer;
import cz.agents.alite.vis.layer.VisLayer;

public class PathLayer extends AbstractLayer {
    public static VisLayer create(final Point2d[] path, final Color color, final int strokeWidth) {
        return new AbstractLayer() {

			@Override
			public void paint(Graphics2D canvas) {
				super.paint(canvas);
				
				canvas.setStroke(new BasicStroke(strokeWidth));
				canvas.setColor(color);
				
				for (int i=0; i < path.length-1; i++) {
					Point2d start = path[i];
					Point2d end = path[i+1];
					canvas.drawLine(
							Vis.transX(start.x), Vis.transY(start.y), 
							Vis.transX(end.x), Vis.transY(end.y));
				}
			}
		};
    }
}
