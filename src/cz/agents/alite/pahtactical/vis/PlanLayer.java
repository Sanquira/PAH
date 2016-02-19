package cz.agents.alite.pahtactical.vis;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Polygon;
import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;

import cz.agents.alite.tactical.util.Point;
import cz.agents.alite.tactical.util.Polygon2d;
import cz.agents.alite.vis.Vis;
import cz.agents.alite.vis.element.Line;
import cz.agents.alite.vis.element.aggregation.LineElements;
import cz.agents.alite.vis.element.implemetation.LineImpl;
import cz.agents.alite.vis.layer.AbstractLayer;
import cz.agents.alite.vis.layer.GroupLayer;
import cz.agents.alite.vis.layer.VisLayer;
import cz.agents.alite.vis.layer.common.CommonLayer;
import cz.agents.alite.vis.layer.terminal.LineLayer;


public class PlanLayer extends CommonLayer {

    public static VisLayer create(final PlanProvider provider,  final Color color, final int strokeWidth) {
		return new AbstractLayer() {
			@Override
			public void paint(Graphics2D canvas) {
				super.paint(canvas);
				
				canvas.setStroke(new BasicStroke(strokeWidth));
				canvas.setColor(color);
				
				List<Point> plan = provider.getPlan();

                if (plan != null) {
                    Point prevPosition = null;
                    for (Point position : plan) {
                        if (prevPosition != null) {
                        	
                        	int x1 = Vis.transX(prevPosition.x);
                        	int y1 = Vis.transY(prevPosition.y);

                        	int x2 = Vis.transX(position.x);
                        	int y2 = Vis.transY(position.y);
                        	
                        	canvas.drawLine(x1, y1, x2, y2);
                        }
                        prevPosition = position;
                    }
                }
			}
		};
    }

    public static interface PlanProvider {
        public List<Point> getPlan();
    }
}

