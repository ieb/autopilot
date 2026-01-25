"""
Base Hardware Simulator
=======================

Base class for hardware simulators that expose Unix socket interfaces.
"""

import socket
import os
import threading
import time
import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Dict, Any
from multiprocessing.managers import DictProxy

logger = logging.getLogger(__name__)


@dataclass
class SimulatorConfig:
    """Base configuration for hardware simulators."""
    socket_path: str
    update_rate_hz: float = 100.0
    
    @property
    def update_interval(self) -> float:
        """Time between updates in seconds."""
        return 1.0 / self.update_rate_hz


class HardwareSimulator(ABC):
    """
    Base class for hardware simulators with Unix socket server.
    
    Subclasses implement the specific protocol (IMU, Actuator, etc.)
    while this base class handles socket management and threading.
    """
    
    def __init__(self, config: SimulatorConfig, shared_state: DictProxy):
        """
        Initialize the simulator.
        
        Args:
            config: Simulator configuration
            shared_state: Shared state dict from multiprocessing.Manager
        """
        self.config = config
        self.state = shared_state
        self.running = False
        
        self._server_socket: Optional[socket.socket] = None
        self._client_socket: Optional[socket.socket] = None
        self._server_thread: Optional[threading.Thread] = None
        self._sim_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        
    def start(self) -> bool:
        """
        Create Unix socket server and start simulation loop.
        
        Returns:
            True if started successfully
        """
        try:
            # Remove existing socket file
            if os.path.exists(self.config.socket_path):
                os.unlink(self.config.socket_path)
                
            # Create server socket
            self._server_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._server_socket.bind(self.config.socket_path)
            self._server_socket.listen(1)
            self._server_socket.settimeout(1.0)  # Allow periodic check for shutdown
            
            self.running = True
            
            # Start accept thread
            self._server_thread = threading.Thread(
                target=self._accept_loop,
                daemon=True,
                name=f"{self.__class__.__name__}-accept"
            )
            self._server_thread.start()
            
            logger.info(f"{self.__class__.__name__} started on {self.config.socket_path}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start {self.__class__.__name__}: {e}")
            return False
            
    def stop(self):
        """Clean shutdown, remove socket file."""
        self.running = False
        
        # Close client connection
        if self._client_socket:
            try:
                self._client_socket.close()
            except Exception:
                pass
            self._client_socket = None
            
        # Close server socket
        if self._server_socket:
            try:
                self._server_socket.close()
            except Exception:
                pass
            self._server_socket = None
            
        # Wait for threads
        if self._server_thread and self._server_thread.is_alive():
            self._server_thread.join(timeout=2.0)
        if self._sim_thread and self._sim_thread.is_alive():
            self._sim_thread.join(timeout=2.0)
            
        # Remove socket file
        if os.path.exists(self.config.socket_path):
            try:
                os.unlink(self.config.socket_path)
            except Exception:
                pass
                
        logger.info(f"{self.__class__.__name__} stopped")
        
    def _accept_loop(self):
        """Accept incoming connections."""
        while self.running:
            try:
                conn, addr = self._server_socket.accept()
                logger.info(f"{self.__class__.__name__} client connected")
                
                with self._lock:
                    # Close existing connection
                    if self._client_socket:
                        try:
                            self._client_socket.close()
                        except Exception:
                            pass
                    self._client_socket = conn
                    
                # Start simulation thread for this connection
                if self._sim_thread and self._sim_thread.is_alive():
                    # Wait for previous thread to finish
                    self._sim_thread.join(timeout=1.0)
                    
                self._sim_thread = threading.Thread(
                    target=self._simulation_loop,
                    daemon=True,
                    name=f"{self.__class__.__name__}-sim"
                )
                self._sim_thread.start()
                
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    logger.error(f"Accept error: {e}")
                break
                
    def _simulation_loop(self):
        """Main simulation loop - runs at configured rate."""
        next_update = time.time()
        
        while self.running and self._client_socket:
            now = time.time()
            
            if now >= next_update:
                try:
                    self._step()
                except BrokenPipeError:
                    logger.info(f"{self.__class__.__name__} client disconnected")
                    break
                except Exception as e:
                    logger.error(f"Simulation step error: {e}")
                    break
                    
                next_update += self.config.update_interval
                
                # Prevent runaway if we're behind
                if next_update < now:
                    next_update = now + self.config.update_interval
            else:
                # Sleep until next update
                sleep_time = next_update - now
                if sleep_time > 0:
                    time.sleep(min(sleep_time, 0.001))  # Max 1ms sleep for responsiveness
                    
    @abstractmethod
    def _step(self):
        """
        Execute one simulation step.
        
        Subclasses implement this to:
        - Read from shared state
        - Generate output message
        - Send to client socket
        - Handle any incoming data
        """
        pass
        
    def send(self, data: bytes):
        """Send data to connected client."""
        with self._lock:
            if self._client_socket:
                self._client_socket.sendall(data)
                
    def recv(self, bufsize: int = 1024) -> Optional[bytes]:
        """Receive data from connected client (non-blocking)."""
        with self._lock:
            if self._client_socket:
                try:
                    self._client_socket.setblocking(False)
                    data = self._client_socket.recv(bufsize)
                    return data if data else None
                except BlockingIOError:
                    return None
                except Exception:
                    return None
                finally:
                    if self._client_socket:
                        self._client_socket.setblocking(True)
        return None
        
    @staticmethod
    def compute_checksum(payload: str) -> str:
        """
        Compute XOR checksum for NMEA-style message.
        
        Args:
            payload: Message content between $ and *
            
        Returns:
            Two-character hex checksum
        """
        checksum = 0
        for c in payload:
            checksum ^= ord(c)
        return f"{checksum:02X}"
        
    @staticmethod
    def verify_checksum(message: str) -> bool:
        """
        Verify checksum of NMEA-style message.
        
        Args:
            message: Full message including $ and *XX
            
        Returns:
            True if checksum is valid
        """
        if not message.startswith('$') or '*' not in message:
            return False
            
        try:
            payload = message[1:message.index('*')]
            expected = message[message.index('*')+1:].strip()
            actual = HardwareSimulator.compute_checksum(payload)
            return actual.upper() == expected.upper()
        except Exception:
            return False
