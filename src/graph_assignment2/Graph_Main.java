/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package graph_assignment2;

/**
 *
 * @author WIN11
 */
public class Graph_Main {

    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) {
        javax.swing.SwingUtilities.invokeLater(new Runnable() {
            public void run() {
                GraphFrame frame = new GraphFrame();
                frame.setVisible(true);
            }
        });
    }
    
}
