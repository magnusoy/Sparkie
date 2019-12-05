package main.java.communication;

import java.io.*;
import java.net.Socket;


/**
 * ClientHandler uses a socket connection from the client.
 * ...
 */
public class ClientHandler extends Thread {

    private final DataInputStream dataInputStream;      // Input from client
    private final DataOutputStream dataOutputStream;    // Output to client
    private final Socket socket;                        // Client socket


    /**
     * ClientHandler constructor. Receives an already
     * connected socket, with assigned input stream and output stream.
     *
     * @param socket           client connection
     * @param dataInputStream  client inputstream
     * @param dataOutputStream client outputstream
     */
    public ClientHandler(Socket socket, DataInputStream dataInputStream,
                         DataOutputStream dataOutputStream) {
        this.dataInputStream = dataInputStream;
        this.dataOutputStream = dataOutputStream;
        this.socket = socket;
    }

    /**
     * This works as an API for the clients. Making them able to
     * communicate, fetch and send data to other resources within
     * the system.
     */
    @Override
    public void run() {
        String received;
        String toReturn;

        while (this.socket.isConnected()) {
            try {
                BufferedReader in = new BufferedReader(new InputStreamReader(this.socket.getInputStream()));

                received = in.readLine();

                switch (received) {
                    case "0":
                        // Do something
                        toReturn = "received";
                        this.dataOutputStream.writeUTF(toReturn);
                        break;

                    case "1":
                        // Do something else
                        toReturn = "received";
                        this.dataOutputStream.writeUTF(toReturn);
                        break;

                    case "2":
                        // Do something else
                        toReturn = "received";
                        this.dataOutputStream.writeUTF(toReturn);
                        break;

                    case "3":
                        toReturn = "Help called";
                        this.dataOutputStream.writeUTF(toReturn);
                        break;

                    default:
                        this.dataOutputStream.writeUTF("Invalid input, type help");
                        break;
                }
            } catch (Exception e) {
                try {
                    this.dataInputStream.close();
                    this.dataOutputStream.close();
                    this.socket.close();
                    System.err.println("Lost connection: " + this.socket);
                    e.printStackTrace();
                    break;
                } catch (IOException ex) {
                    ex.printStackTrace();
                }
            }
        }
        try {
            this.dataInputStream.close();
            this.dataOutputStream.close();

        } catch (IOException ioe) {
            ioe.printStackTrace();
        }
    }
}