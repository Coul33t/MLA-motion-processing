#ifndef __MLA_INPUT__
#define __MLA_INPUT__

#include "SDL.h"

class Input {
public:
	Input();
	virtual ~Input();
	void EventUpdate();

	int GetX() const;
	int GetY() const;
	int GetXRel() const;
	int GetYRel() const;

	bool End() const;
	bool GetKey(const SDL_Scancode key) const;
	bool GetMouseButton(const Uint8 button) const;
	bool MouseMoving() const;
	void DisplayCursor(bool disp) const;
	void TrapMouse(bool trap) const;

private:
	SDL_Event m_events;
	bool m_keys[SDL_NUM_SCANCODES];
	bool m_mouse_buttons[8];

	int m_x;
	int m_y;
	int m_x_rel;
	int m_y_rel;

	bool m_end;
};

#endif //__MLA_INPUT__