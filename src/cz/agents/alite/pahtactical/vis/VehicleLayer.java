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
import dubins.Point2dYaw;

public class VehicleLayer extends AbstractLayer {
    public static VisLayer create(final Polygon2d footprint, final Point2dYaw state, final Color color) {
    	return new AbstractLayer() {
    		
			@Override
			public void paint(Graphics2D canvas) {
				super.paint(canvas);

				Polygon2d rotatedFootprint = footprint.getRotated(new Point2d(0,0), state.getYaw());
				Polygon2d translatedFootprint = rotatedFootprint.getTranslated(state.getPosition());
				
				Point2d[] points = translatedFootprint.getPoints();
                
				int[] xpoints = new int[points.length];
				int[] ypoints = new int[points.length];
				
				for (int i = 0; i < points.length; i++) {
					xpoints[i] = Vis.transX(points[i].x);
					ypoints[i] = Vis.transY(points[i].y);
				}
				
				canvas.setStroke(new BasicStroke(1));
				
				canvas.setColor(color);
				canvas.drawPolygon(new Polygon(xpoints, ypoints, points.length));				
				
				canvas.fillOval(Vis.transX(state.x)-2, Vis.transY(state.y)-2, 4, 4);
			} 
		};
    }
}
