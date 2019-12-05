package main.java.communication;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;

/**
 * Server works as the core of the system.
 * Handling calls from clients and directing
 * the tasks to different processes and threads.
 */
public class Server {

    private ServerSocket serverSocket;    // Initialize socket

    /**
     * Server constructor. Initialize a server socket and
     * assign the given port to the socket.
     *
     * @param port, socket port
     */
    public Server(int port) {
        // Socket port
        try {
            this.serverSocket = new ServerSocket(port);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Start the server and creates a new thread for
     * every new client connecting. Assigning a new
     * clientHandler so the client can GET and POST
     * data.
     *
     * @throws IOException, Exception
     */
    public void start() throws IOException {
        while (true) {
            Socket socket = null;

            try {
                socket = serverSocket.accept();
                System.out.printf("A new client is connected: %s%n", socket);

                DataInputStream dataInputStream = new DataInputStream(socket.getInputStream());
                DataOutputStream dataOutputStream = new DataOutputStream(socket.getOutputStream());

                // Assigning new thread for this client
                Thread clientThread = new ClientHandler(socket,
                        dataInputStream,
                        dataOutputStream);

                clientThread.start();
            } catch (Exception e) {
                assert socket != null;
                socket.close();
                e.printStackTrace();
            }
        }
    }
}