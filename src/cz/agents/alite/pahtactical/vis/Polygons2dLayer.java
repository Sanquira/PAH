package cz.agents.alite.pahtactical.vis;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Polygon;
import java.util.ArrayList;
import java.util.Collection;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import cz.agents.alite.tactical.util.Polygon2d;
import cz.agents.alite.vis.Vis;
import cz.agents.alite.vis.element.Line;
import cz.agents.alite.vis.element.aggregation.LineElements;
import cz.agents.alite.vis.element.implemetation.LineImpl;
import cz.agents.alite.vis.layer.AbstractLayer;
import cz.agents.alite.vis.layer.VisLayer;
import cz.agents.alite.vis.layer.terminal.LineLayer;

public class Polygons2dLayer extends AbstractLayer {
    public static VisLayer create(final Collection<Polygon2d> polygons,  final Color color, final int strokeWidth) {
    	return new AbstractLayer() {
    		
			@Override
			public void paint(Graphics2D canvas) {
				super.paint(canvas);
				

				
				for (Polygon2d polygon : polygons) {
					Point2d[] points = polygon.getPoints();
	                
					int[] xpoints = new int[points.length];
					int[] ypoints = new int[points.length];
					
					for (int i = 0; i < points.length; i++) {
						xpoints[i] = Vis.transX(points[i].x);
						ypoints[i] = Vis.transY(points[i].y);
					}
					
					canvas.setStroke(new BasicStroke(strokeWidth));

					canvas.setColor(color.brighter().brighter());
					canvas.fillPolygon(new Polygon(xpoints, ypoints, points.length));
					
					canvas.setColor(color);
					canvas.drawPolygon(new Polygon(xpoints, ypoints, points.length));
				} 
			}
		};
    }
}
