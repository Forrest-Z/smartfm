package com.smartfm.phoneui;

public class StationDoesNotExistException extends RuntimeException {

	private static final long serialVersionUID = -2583561921266309376L;

	public StationDoesNotExistException() {
	}

	public StationDoesNotExistException(String message) {
		super(message);
	}

	public StationDoesNotExistException(Throwable cause) {
		super(cause);
	}

	public StationDoesNotExistException(String message, Throwable cause) {
		super(message, cause);
	}

}
